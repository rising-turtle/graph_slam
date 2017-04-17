/*
 * Oct. 6, 2016, David Z
 *
 * test whether the imu measurement is correct 
 *
 *
 * */

#include "imu_MEMS.h"
#include "SR_reader_cv.h"
#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <std_msgs/Float32MultiArray.h>
// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>

using namespace gtsam;
using namespace std; 

string g_file_dir;          // camera data folder 
string g_syn_camera_imu;    // file contain camera time data 
string g_imu_data_dir;      // where to read the imu data 
int g_end_n;                
int g_imu_id_first_camera;  // imu id corresponds to the first camera data 

void init_parameters(); 
void show_imu_measurement_with_camera(); 

vector<int> load_syn_camera_time(); // load time sequence for camera data  


ros::Publisher g_imu_pub;
void writeToLogAndPublish(ostream&, Pose3&);


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_imu_MEMS"); 
  ros::NodeHandle n; 
  g_imu_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); 
  init_parameters(); 
  printf("after init parameters\n");
  show_imu_measurement_with_camera(); 
  

  return 0; 
}


void show_imu_measurement_with_camera()
{ 
  CSReadCV r4k;  // sr4k reader 
  // extract intensity and depth img for each sr_data 
  cv::Mat i_img, d_img; // 
  
  CImuMEMS* imu_measure = new CImuMEMS;  // imu preintegration interface using MEMS imu 
  printf("after imu_measure construction\n");
  if(!imu_measure->readImuData(g_imu_data_dir))
  {
    ROS_ERROR("failed to read imu data from %s", g_imu_data_dir.c_str()); 
    return ; 
  }
  imu_measure->computePriorBias(); // compute bias first 

  vector<int> camera_time_set = load_syn_camera_time(); 

  // output imu measurement 
  size_t found = g_file_dir.find_last_of("/\\");
  string dataname = g_file_dir.substr(found+1) + "_inertial.log"; 
  ofstream ouf(dataname.c_str());

  // imu data index 
  int imu_next_id = g_imu_id_first_camera; // imu_camera_syn_id 
  ROS_WARN("imu_id with the first camera is %d", imu_next_id);
  double time_elapsed = 0; 
  NavState curr_pose; 

  ROS_ERROR("g_end_n is %d", g_end_n); 

  for(int i=1; i<g_end_n; ++i)
  {
    // get current img 
    stringstream ss; 
    ss<<g_file_dir<<"/d1"<<"_"<<setfill('0')<<setw(4)<<i<<".bdat"; 

    // while(r4k.getCurrentFrameCV(i_img, d_img))  // get current img 
    if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
    {
      // show them and get imu measurement in this time 
      time_elapsed = (i >= camera_time_set.size())?34:camera_time_set[i]; 
      if(time_elapsed < 20 ) time_elapsed = 34; 
      else if(time_elapsed > 100 && time_elapsed < 150 ) time_elapsed *= 2; 
      else if(time_elapsed >= 150) time_elapsed *= 1.5; 
      // time_elapsed = 34; 
      imu_next_id += (int)(((float)time_elapsed)/10.+0.5); // 10 ms/frame
      // imu_next_id += (int)(((float)time_elapsed)/3.33+0.5); // 3.33 ms/frame
      // imu_next_id += (int)(((float)time_elapsed)/5.+0.5); // 5 ms/frame
      // imu_next_id += (int)(((float)time_elapsed)/6.7+0.5); // 6.7 ms/frame
      // imu_next_id += (int)(((float)time_elapsed)/9.+0.5); // 6.7 ms/frame

      curr_pose = imu_measure->predictNext(imu_next_id); 
      imu_measure->resetPreintegrationAndBias(); 
      imu_measure->setState(curr_pose); 
      
      Pose3 cp = curr_pose.pose(); 
      writeToLogAndPublish(ouf, cp);
      
      cv::imshow("intensity_img", i_img); 
      cv::waitKey(50); 
    }else{
    
      ROS_INFO("failed to read more camera data!"); 
      break; 
    }
    
    if(!ros::ok())
    { break;}

  }
} 


void init_parameters()
{
  ros::NodeHandle np("~"); 
  // np.setParam("sr_data_file_dir", "/media/david/work/work/data/SLAM/SR4000/imu_fuse/frames_creep_ob1"); 
  // np.setParam("sr_start_frame", 1); 
  // np.setParam("sr_end_frame", 2070);
  np.setParam("sr_new_file_version", false);
  // np.setParam("imu_file_dir", "/media/david/work/work/data/SLAM/SR4000/imu_fuse/imu_creep_ob1.dat"); 
  // np.setParam("camera_syn_data", "/media/david/work/work/data/SLAM/SR4000/imu_fuse/ftiming_creep_ob1.dat");

  // nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  np.param("sr_data_file_dir", g_file_dir, string("/home/davidz/work/data/SwissRanger4000/exp/dataset_82")); // default parameters 
  // np.param("sr_data_prefix", g_file_pre, string("d1")); 
  // np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  // np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_end_n, 500); 
  np.param("imu_file_dir", g_imu_data_dir, string("")); 
  np.param("camera_syn_data", g_syn_camera_imu, string(""));
  np.param("imu_id_first_cam", g_imu_id_first_camera, 50);

  ROS_WARN("data_file: %s imu_file_dir %s camera_syn_data %s", g_file_dir.c_str(), g_imu_data_dir.c_str(), g_syn_camera_imu.c_str() );

}


void writeToLogAndPublish(ostream& ouf, Pose3& p)
{
  Vector3 rpy = p.rotation().rpy();
  ouf<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<rpy(0)<<" "<<rpy(1)<<" "<<rpy(2)<<endl;
  
  std_msgs::Float32MultiArray msg; 
  msg.data.resize(3); 
  msg.data[0] = rpy(0); msg.data[1] = rpy(1); msg.data[2] = rpy(2); 
  g_imu_pub.publish(msg); 
  ROS_WARN("%s: publish rpy %f %f %f",__FILE__, rpy(0), rpy(1), rpy(2));
}

vector<int> load_syn_camera_time() // load time sequence for camera data  
{
  vector<int> times; 
  ifstream inf(g_syn_camera_imu.c_str());
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open syn_camera_imu file %s", g_syn_camera_imu.c_str()); 
    return times; 
  }
  
  int t; 
  while(!inf.eof())
  {
    inf >> t; 
    times.push_back(t); 
  }
  return times; 
}

