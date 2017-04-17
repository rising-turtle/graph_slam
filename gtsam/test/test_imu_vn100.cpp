/*
 *  analysis the imu_vn100 data 
 *
 * */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include "std_msgs/Float32MultiArray.h"
#include "imu_vn100.h"
#include "SR_reader_cv.h"
#include "camera_node.h"
#include "gtsam_graph.h"
#include <opencv2/opencv.hpp>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

using namespace std; 
using namespace gtsam;
using namespace CG;

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 

#define D2R(d) (((d)*M_PI)/180)
#define R2D(r) (((r)*180)/M_PI)

string fname("/home/david/work/data/sr4k/imu_bdat/etas_f5/imu_v100.log"); 
ros::Publisher euler_pub; 

int test_rpy();           // output rpy 
int test_rpy_with_img();  // output rpy matched with img 
int test_rpy_with_gtsam(); // output rpy result from gtsam 

int main(int argc, char* argv[])
{
  // CImuBase::getParam();  // call it before ros::init to avoid occasionally eigen crash, if you do not understand, comment this you will see what's happening

  ros::init(argc, argv, "test_imu_vn100"); 
  ros::NodeHandle n; 
  euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); 

 // test_rpy(); 
 // test_rpy_with_img();
 test_rpy_with_gtsam(); 

  return 0; 
}

int test_rpy_with_gtsam()
{
  ros::NodeHandle nh("~");
  
  string img_time_file("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k/timestamp.log");
  string img_dir("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k"); 

  nh.param("imu_file", fname, fname); 
  nh.param("img_time_file", img_time_file, img_time_file); 
  nh.param("img_dir", img_dir, img_dir); 

  // load img timestamps 
  ifstream inf(img_time_file.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open file %s", img_time_file.c_str()); 
    return -1; 
  }

  CGraphGT g; 

  // load imu file 
  gtsam::imuBias::ConstantBias prior_bias; 
  double dt = 0.005; // 0.005 - 200 hz // 0.025 40hz
  CImuVn100* imu = new CImuVn100(dt, prior_bias); 
  if(!imu->readImuData(fname))
  {
    ROS_ERROR("failed to load imu data from file %s", fname.c_str()); 
    return -1; 
  }
  
  double last_imu_timestamp = imu->getLastTimeStamp(); 

  CSReadCV r4k ; // read sr4k data 
  cv::Mat i_img, d_img; 
  int img_id; 
  double img_timestamp; 
  Eigen::Vector3d rpy; 
  Eigen::Vector3d i_rpy;  // integrated r p y
  std_msgs::Float32MultiArray msg; 
  msg.data.resize(3); 
  bool firstT = true; 
  int kk = 0; 

  while(!inf.eof() && ros::ok())
  {
    inf >> img_id >> img_timestamp; 
    
    // if(++kk > 110) break;

    // when the imu stops before the camera sequence 
    if(img_timestamp >= last_imu_timestamp) 
    {
      ROS_INFO("img_timestamp : %f >= last_imu_timestamp %f exit", img_timestamp, last_imu_timestamp); 
      break;
    } 

    // read img 
    stringstream ss; 
    ss << img_dir<<"/d1_"<<setfill('0')<<setw(7)<<img_id<<".bdat"; 
    if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
    {
      CCameraNode* pNewNode = new CCameraNode(); 
      pNewNode->m_id = g.m_graph_map.size(); 
      pNewNode->m_seq_id = img_id; 
      if(firstT)
      {
        imu->setStartPoint(img_timestamp);
        g.firstNode(pNewNode, false); 
        firstT = false; 
      }else
      {
        g.m_graph_map[pNewNode->m_id] = pNewNode; 
        
        // get rpy from imu 
        imu->getRPYAt(img_timestamp, rpy); 

        // get navstate from imu preintegration 
        NavState cur_p = imu->predictNext(img_timestamp); 
               
        int cur_node_id = pNewNode->m_id; 
        // add imu measurement into graph 
        cur_node_id = pNewNode->m_id; 
        PreintegratedCombinedMeasurements* preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu->mp_combined_pre_imu); 
        CombinedImuFactor imu_factor(X(cur_node_id-1), V(cur_node_id-1), 
                                    X(cur_node_id),    V(cur_node_id), 
                            B(cur_node_id-1), B(cur_node_id), *(preint_imu_combined));
        g.mp_fac_graph->add(imu_factor); 
        g.mp_new_fac->add(imu_factor); 
        g.addToGTSAM(cur_p, cur_node_id, true); 

        // preint_imu_combined->print("integrate Measurement COV: "); 
        // if(++kk >=3) break;

        // add gps factor 
        noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,0.001);
        GPSFactor gps_factor(X(cur_node_id),
                           Point3(0,  // N,
                                  0,  // E,
                                  0), // D,
                           correction_noise);
        g.mp_fac_graph->add(gps_factor); 
        g.mp_new_fac->add(gps_factor); 

        g.optimizeGraphIncremental(); 

        // imu->resetPreintegrationAndBias();  // no new bias avaliable 
        imu->resetPreintegrationAndBias(g.mp_node_values->at<imuBias::ConstantBias>(B(cur_node_id)));  // no new bias avaliable 
        
        cur_p = NavState(g.mp_node_values->at<Pose3>(X(cur_node_id)), g.mp_node_values->at<Vector3>(V(cur_node_id))); 
        imu->setState(cur_p); 

        Pose3 p = cur_p.pose(); 
        i_rpy = p.rotation().rpy();

        // publish rpy 
        // msg.data[0] = D2R(rpy(0)); msg.data[1] = D2R(rpy(1)); msg.data[2] = D2R(rpy(2)); 
        msg.data[0] = i_rpy(0); msg.data[1] = i_rpy(1); msg.data[2] = i_rpy(2); 
        euler_pub.publish(msg); 

        ROS_WARN("Kalman result: %f %f %f", rpy(0), rpy(1), rpy(2)); 
        ROS_ERROR("GTSAM result: %f %f %f", R2D(i_rpy(0)), R2D(i_rpy(1)), R2D(i_rpy(2))); 

        // show the img 
        cv::imshow("intensity_img", i_img); 
        cv::waitKey(30);

        // store the intermidiate trajectory result 
        if(cur_node_id % 100 == 0)
        {
          stringstream ss; 
          ss <<"./tmp/"<<cur_node_id<<"_traj.ply";
          g.trajectoryPLY(ss.str(), BLUE); 
        }
      }
    }
  }
  {
      stringstream ss; 
      ss <<"./tmp/whole_traj.ply";
      g.trajectoryPLY(ss.str(), RED); 
  }

  delete imu; 

}

int test_rpy_with_img()
{
  ros::NodeHandle nh("~");
  
  string img_time_file("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k/timestamp.log");
  string img_dir("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k"); 

  nh.param("imu_file", fname, fname); 
  nh.param("img_time_file", img_time_file, img_time_file); 
  nh.param("img_dir", img_dir, img_dir); 

  // load img timestamps 
  ifstream inf(img_time_file.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open file %s", img_time_file.c_str()); 
    return -1; 
  }

  // load imu file 
  gtsam::imuBias::ConstantBias prior_bias; 
  double dt = 0.005; // 200 hz
  CImuVn100* imu = new CImuVn100(dt, prior_bias); 
  if(!imu->readImuData(fname))
  {
    ROS_ERROR("failed to load imu data from file %s", fname.c_str()); 
    return -1; 
  }
  
  CSReadCV r4k ; // read sr4k data 
  cv::Mat i_img, d_img; 
  int img_id; 
  double img_timestamp; 
  Eigen::Vector3d rpy; 
  Eigen::Vector3d i_rpy;  // integrated r p y
  std_msgs::Float32MultiArray msg; 
  msg.data.resize(3); 
  bool firstT = true; 
  
  while(!inf.eof() && ros::ok())
  {
    inf >> img_id >> img_timestamp; 
    
    // read img 
    stringstream ss; 
    ss << img_dir<<"/d1_"<<setfill('0')<<setw(7)<<img_id<<".bdat"; 
    if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
    {
      if(firstT)
      {
        imu->setStartPoint(img_timestamp);
        firstT = false; 
      }
      
      // get rpy from imu 
      imu->getRPYAt(img_timestamp, rpy); 
      
      // get navstate from imu preintegration 
      NavState cur_p = imu->predictNext(img_timestamp); 
      imu->resetPreintegrationAndBias();  // no new bias avaliable 
      imu->setState(cur_p); 
      
      Pose3 p = cur_p.pose(); 
      i_rpy = p.rotation().rpy();

      // publish rpy 
      // msg.data[0] = D2R(rpy(0)); msg.data[1] = D2R(rpy(1)); msg.data[2] = D2R(rpy(2)); 
      msg.data[0] = i_rpy(0); msg.data[1] = i_rpy(1); msg.data[2] = i_rpy(2); 
      euler_pub.publish(msg); 

      ROS_WARN("Kalman result: %f %f %f", rpy(0), rpy(1), rpy(2)); 
      ROS_ERROR("GTSAM result: %f %f %f", R2D(i_rpy(0)), R2D(i_rpy(1)), R2D(i_rpy(2))); 

      // show the img 
      cv::imshow("intensity_img", i_img); 
      cv::waitKey(30);
    }
  }
  delete imu; 
}


int test_rpy()
{
  ifstream inf(fname.c_str()); 
  
  double t; 
  float ax, ay, az, gx, gy, gz, yaw, pitch, roll; 
  float g_yaw, g_pitch, g_roll; 
  std_msgs::Float32MultiArray msg; 
  msg.data.resize(3); 
  int cnt = 0; 
  while(!inf.eof() && ros::ok())
  {
    
    inf>>t>>ax>>ay>>az>>gx>>gy>>gz>>yaw>>pitch>>roll; 
    
    if(cnt++ == 0)
    {
      g_yaw = yaw; 
      g_pitch = pitch; 
      g_roll = roll; 
      ROS_INFO("gyaw %f gpitch %f groll %f", g_yaw, g_pitch, g_roll); 
    }
    
    msg.data[0] = D2R(roll - g_roll);
    msg.data[1] = D2R(pitch - g_pitch); 
    msg.data[2] = D2R(yaw - g_yaw); 

    ROS_INFO("msg out %f %f %f ", roll-g_roll, pitch-g_pitch, yaw - g_yaw); 

    euler_pub.publish(msg); 
    ros::spinOnce(); 
    usleep(50*1000); 
  }

  inf.close();
}


