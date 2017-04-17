/*
 * test plane propagation based given the result from VO 
 * 
 * Jan.31, 2017, David Z 
 *
 * */

#include <ros/ros.h> 
#include <sstream>
#include <string>
#include "SR_reader_cv.h"
#include "sparse_feature_vo.h"
#include "camera_node.h"
#include "cam_model.h"
#include "gtsam_graph.h"
#include "gt_parameter.h"
#include "plane_node.h"
#include "plane.h"
#include "plane_set.h"
#include "display_many_imgs.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 
using symbol_shorthand::L; // plane landmark (nv, d)

void init_parameters(); 
void test_plane_propogate(); 

void copyMat(cv::Mat& f, cv::Mat& t);

string g_file_dir; 
string g_file_pre; 
string g_file_suf;
string g_data_name;

bool g_chi2_test = false;   // whether to use Chi2 test to delete VRO edges 
bool g_plane_factor = false; // whether to add plane factors into graph 
int g_f_start; 
int g_f_end;

string g_imu_file; // imu measurement file 
string g_imu_time_file; // sr timestamp file, which is used to synchronize between camera data and imu data 
string g_vro_results_file; // where the vro results are stored 

void display_img(string title, cv::Mat& img1, cv::Mat& img2);
void testEigen();     // Eigen inverse not correct when the entries of the matrix are too small 
void testProject();   

int main(int argc, char* argv[])
{
  // CImuBase::getParam(); 
  ros::init(argc, argv, "test_plane_propogate"); 
  ros::NodeHandle n; 
  
  ROS_INFO("at line %s before test_with_sr4k", __FILE__);

  test_plane_propogate(); 
  // testEigen();
  // testProject(); 

  return 0; 
}

void test_plane_propogate()
{
  
  // parameters 
  init_parameters(); 
  // std::string fname = "/home/david/.ros/vro_results_backup.log";   
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  CCameraNode::set_cam_cov(sr4k); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;
  sr4k.setDepthScale(depth_scale); 
  CamModel::updategCamModel(sr4k); 
  
  CSReadCV r4k; 

  // set graph strcuture 
  CGraphGT gt_graph; 
   // VRO result 
  gt_graph.readVRORecord(g_vro_results_file); 
  // gt_graph.setWorld2Original(D2R(CGTParams::Instance()->m_initial_pitch));
  // gt_graph.setCamera2IMU(0); 

  cv::Mat rgb1 = cv::Mat(sr4k.m_rows, sr4k.m_cols, CV_8UC3); 
  cv::Mat rgb2 = cv::Mat(rgb1.size(), CV_8UC3);

  for(int i=0; i<gt_graph.mv_vro_res.size(); i+=27)
  {
    MatchingResult* mr = gt_graph.mv_vro_res[i]; 
    if(mr->edge.informationMatrix(0,0) == 10000) continue; 
    // if(mr->edge.id2 != 1177 || mr->edge.id1 != 1170) continue;

    ROS_INFO("i = %d get a new mr", i);

    // extract intensity and depth img for each sr_data 
    cv::Mat img1, dpt1; // 
    cv::Mat img2, dpt2; 

    // find two images 
    {
      stringstream ss; 
      ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<mr->edge.id2<<"."<<g_file_suf;  // setw(4)
      if(!r4k.readOneFrameCV(ss.str(), img2, dpt2))
      {
        return ;
      }
    }
    {
      stringstream ss; 
      ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<mr->edge.id1<<"."<<g_file_suf;  // setw(4)
      if(!r4k.readOneFrameCV(ss.str(), img1, dpt1))
      {
        return; 
      }
    }
    
    stringstream ss; 
    ss << "from "<<mr->edge.id1<<" to "<<mr->edge.id2; 

    // findout pose and covariance 
    Pose3 inc_pose(mr->edge.transform.matrix()); 
    Matrix6 S_p = mr->edge.informationMatrix.inverse(); 

    // find planes in img1 
    CPlaneNode* p1 = new CPlaneNode(); 
    // PlaneNode* p2 = new PlaneNode(); 
    
    ROS_INFO("before p1 extract planes:"); 

    int np1 = p1->extractPlanes(img1, dpt1, &sr4k); 
    
    // p1->testEigen(); 

    ROS_INFO("%s after p1 extract planes, frame1 has %d planes", __FILE__, np1);
    
    vector<int> newly_added; 

    CPlaneNode* p2 = gt_graph.predictPlaneNode(p1, img2, dpt2, &inc_pose, S_p, &newly_added); 
    
    ROS_INFO("%s after p2 extract planes, frame2 has %d planes", __FILE__, np1);

    // p2->testPtOnPlane(); 

    // color the points 
   
      copyMat(img1, rgb1); 
      copyMat(img2, rgb2);

      int c = GREEN; 
      int pre_n = p1->mv_planes.size(); 
      for(int i=0; i<p1->mv_planes.size(); i++)
      {
        CPlane* p = p1->mv_planes[i];
        ROS_INFO("p1 plane %d has points %d nv: %f %f %f d: %f", i, p1->mv_indices[i].size(), p->nx_, p->ny_, p->nz_, p->d1_);
        markColor(rgb1, p1->mv_indices[i], static_cast<COLOR>(c++%5)); 
      }

      c = GREEN; 
      for(int i=0; i<p2->mv_planes.size(); i++)
      {
        CPlane * p = p2->mv_planes[i];
        ROS_INFO("p2 plane %d has points %d nv: %f %f %f d: %f", i, p2->mv_indices[i].size(), p->nx_, p->ny_, p->nz_, p->d1_);

        // if(i >= pre_n)
        {
          // markColor(rgb2, p2->mv_indices[i], PURPLE); 
          // ROS_INFO("add purple pts with %d", p2->mv_indices[i].size());
        }
        // else
          markColor(rgb2, p2->mv_indices[i], static_cast<COLOR>(c++%5)); 
      }   
      markColor(rgb2, newly_added, RED); 

    display_img(ss.str(), rgb1, rgb2); 

    delete p1; 
    delete p2; 
  }
  return ;
}

void display_img(string s,  cv::Mat& rgb1, cv::Mat& rgb2)
{
  // cv::Mat rgb1 = cv::Mat(img1.size(), CV_8UC3); 
  // cv::Mat rgb2 = cv::Mat(img2.size(), CV_8UC3); 

  // copyMat(img1, rgb1); 
  // copyMat(img2, rgb2); 

  // display it 
  IplImage Ipl1 = rgb1; 
  IplImage Ipl2 = rgb2; 
  cvShowManyImages(s.c_str(), 2, &Ipl1, &Ipl2); 
}


void init_parameters()
{
  ros::NodeHandle np("~"); 
  // np.setParam("sr_data_file_dir", "/media/david/work/work/data/SLAM/SR4000/dataset_82"); 
  // np.setParam("sr_start_frame", 1); 
  // np.setParam("sr_end_frame", 2270);
  // np.setParam("sr_new_file_version", false);

  // nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  np.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k")); // default parameters 
  np.param("sr_data_prefix", g_file_pre, string("d1")); 
  np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_f_end, 500); 
  np.param("sr_data_name", g_data_name, string("etas_f5"));
  np.param("vro_results_file", g_vro_results_file, string("/home/david/.ros/vro_results/etas_f5_vro_results.log")); 
}

void copyMat(cv::Mat& f, cv::Mat& t)
{
  if(f.type() == t.type() )
  {
    t = f.clone();
    return ;
  }

  int color_index, grey_index; 
  int rgb_size = 3; 
  int grey_size =1; 
  if(f.type() == CV_8UC1 && t.type() == CV_8UC3)
  {
    for(int r=0; r<t.rows; r++)
    for(int c=0; c<t.cols; c++)
    {
      color_index = (r*t.cols + c)*rgb_size; 
      grey_index = (r*t.cols + c)*grey_size; 
      t.at<uint8_t>(color_index + 2) = f.at<uint8_t>(grey_index);  // r
      t.at<uint8_t>(color_index + 1) = f.at<uint8_t>(grey_index);  // g 
      t.at<uint8_t>(color_index + 0) = f.at<uint8_t>(grey_index);  // b
    }
  }else
  {
    ROS_ERROR("%s TODO: copyMat with other Mat types", __FILE__); 
  }
  return ;
}


void testEigen()
{
  
   Eigen::Matrix<double, 4, 4 > C; 
 C<< 3.56733e-07,  1.11311e-08,   2.5493e-09,  -7.8651e-08,
     1.11311e-08,  4.37023e-09, -1.35016e-08,  2.02329e-08,
     2.5493e-09, -1.35016e-08,  4.58673e-08, -7.71521e-08,
     -7.8651e-08,  2.02329e-08, -7.71521e-08,  1.69723e-07; 
    Eigen::Matrix<double, 3, 3> C1 = C.block<3,3>(0,0); 
    
    // C1 << 3.56733e-07,  1.11311e-08,   2.5493e-09,
    //      1.11311e-08,  4.37023e-09, -1.35016e-08,
    //       2.5493e-09,  -1.35016e-08,  4.58673e-08;
    Eigen::Matrix<double, 3, 3> CN = C1.inverse(); 
    Eigen::Matrix<double, 3, 3> T = C1*CN ; 
    cout <<"C1: "<<endl<<C1<<endl;
    cout <<"C1N: "<<endl<<CN<<endl;
    cout <<"C1*C1N: "<<endl<<T<<endl;

    CPlane p; 
    p.saveCP(C); 
    // p.testEigen(); 
    CPlaneNode* pn = new CPlaneNode(); 
    pn->mv_planes.push_back(&p); 
    pn->testEigen(); 


  init_parameters(); 
    CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
    CCameraNode::set_cam_cov(sr4k); 
    sr4k.z_offset = 0.015;  // this is only for sr4k 
    float depth_scale = 0.001;
    sr4k.setDepthScale(depth_scale); 
    CamModel::updategCamModel(sr4k); 
 
    CPlaneNode* p1 = new CPlaneNode(); 
    // PlaneNode* p2 = new PlaneNode(); 

     CSReadCV r4k; 

    cv::Mat img1, dpt1; // 

    {
      stringstream ss; 
      ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<204<<"."<<g_file_suf;  // setw(4)
      r4k.readOneFrameCV(ss.str(), img1, dpt1); 
    }
   
    int np1 = p1->extractPlanes(img1, dpt1, &sr4k); 
    p1->testEigen(); 
}


void testProject()
{
    init_parameters(); 
    CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
    CCameraNode::set_cam_cov(sr4k); 
    sr4k.z_offset = 0.015;  // this is only for sr4k 
    float depth_scale = 0.001;
    sr4k.setDepthScale(depth_scale); 
    CamModel::updategCamModel(sr4k); 
 
    CSReadCV r4k; 
    cv::Mat img1, dpt1; // 

    {
      stringstream ss; 
      ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<204<<"."<<g_file_suf;  // setw(4)
      r4k.readOneFrameCV(ss.str(), img1, dpt1); 
    }
   
    ofstream ouf("testProject.log"); 
    double px, py, pz;  float uf, vf;
    int uj, vj; 
    for(int v = 0; v < img1.rows; v++)
      for(int u = 0; u<img1.cols; u++)
      {
        pz = dpt1.at<unsigned short>(v, u) * sr4k.m_z_scale; 
        sr4k.convertUVZ2XYZ(u,v, pz, px,py, pz); 
        
        sr4k.convertXYZ2UV(px, py, pz, uf, vf); 
        uj = (int)(uf+0.5); 
        vj = (int)(vf+0.5); 
        
        if(uj != u || vj != v)
        {
          ouf <<u<<" "<<v<<" "<<pz<<" "<<px<<" "<<py<<" "<<uf<<" "<<vf<<" "<<uj<<" "<<vj<<endl;
        }
        
      }
}

