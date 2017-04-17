/*
 * Feb. 13, 2017 David Z 
 * 
 * show registration result between two frames (i,j)  given Tij 
 *
 * */


#include <ros/ros.h>
#include <string> 
#include <tf/tf.h>
#include <iostream>
#include <map>
#include <pcl/common/transforms.h>
#include "sparse_feature_vo.h"
#include "SR_reader_cv.h"
#include "camera_node_ba.h"
#include "glob_def.h"
#include "pc_from_image.h"
#include "vtk_viewer.h"
#include "gtsam_graph.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "plane.h"
#include "plane_node.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/OrientedPlane3.h>


// using namespace cv; 
using namespace std; 
using namespace gtsam; 

#define R2D(r) (((r)*180.)/M_PI)

 void init_parameters(); 
 void testTwoFrameMatch(); 



int main(int argc, char* argv[])
{
   ros::init(argc, argv, "view_registration"); 
   ros::NodeHandle n;
   init_parameters(); 
   testTwoFrameMatch(); 
  return 0; 
}

void testTwoFrameMatch()
{
  int idi = 2686;
  int idj = 2688; 
  string g_file_dir; 
  string g_vro_results_file; 
  ros::NodeHandle nh("~"); 
  
  nh.param("from_idi", idi, idi); 
  nh.param("to_idj", idj, idj); 
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k")); // default parameters
  nh.param("vro_results_file", g_vro_results_file, string("vro_results.log_hybrid")); 

   // set graph strcuture 
  CGraphGT gt_graph; 
  ROS_INFO("before read vro results!"); 
   // VRO result 
  gt_graph.readVRORecord(g_vro_results_file); 
  ROS_INFO("after read vro results!");  

    // VRO 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  sr4k.setDepthScale(0.001); 
  CCameraNode::set_cam_cov(sr4k); 
  CCameraNode::gb_dis_match_point = true; // whether to see the result of
  bool get_start = false; 

  int i,j; 
  for(int k=0; k<gt_graph.mv_vro_res.size() && ros::ok(); k++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[k]; 
    if((!get_start) && ( pm->edge.id2 != idj || pm->edge.id1 != idi)) continue; 
    
    get_start = true;   
    i = pm->edge.id1; 
    j = pm->edge.id2; 
    
    if(pm->edge.informationMatrix(0,0) == 10000)
      continue; 

    Matrix4 TMij = pm->final_trafo.cast<double>(); 
    // getframe 
    stringstream sni, snj; 
    sni << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<i<<".bdat"; 
    snj << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<j<<".bdat"; 
    
    cout <<"Result from "<<i<<" to "<<j<<endl;

    // CSReader r4k; 
    CSReadCV r4k; 

    // generate imgs and dpts 
    cv::Mat tar_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
    cv::Mat src_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
    cv::Mat src_cv_i_img; 
    cv::Mat tar_cv_i_img; 

    if(!r4k.readOneFrameCV(snj.str().c_str(), src_cv_i_img, src_cv_d_img))
    {
      ROS_INFO("%s failed to read file %s", __FILE__, snj.str().c_str()); 
      return ;
    }
    if(!r4k.readOneFrameCV(sni.str().c_str(), tar_cv_i_img, tar_cv_d_img))
    {
      ROS_INFO("%s failed to read file %s", __FILE__, sni.str().c_str()); 
      return ;
    }

    // show them in point cloud 
    CloudPtr pci(new Cloud); 
    CloudPtr pcj(new Cloud); 
    CloudPtr pcj_ni(new Cloud); 
    generatePointCloud(tar_cv_i_img, tar_cv_d_img, 0.001, sr4k, *pci);
    generatePointCloud(src_cv_i_img, src_cv_d_img, 0.001, sr4k, *pcj);
    pcl::transformPointCloud(*pcj, *pcj_ni,  TMij.cast<float>()); // mr.final_trafo); 
  
    // markColor(*pci, pni->mv_indices[best_i], GREEN); 
    // markColor(*pcj_ni, pnj->mv_indices[best_j], RED); 
    markColor(*pci,  GREEN); 
    markColor(*pcj_ni, RED); 
 
    *pci+= *pcj_ni;

    CVTKViewer<pcl::PointXYZRGBA> v;
    // v.getViewer()->addCoordinateSystem(0.2, 0, 0); 
    v.addPointCloud(pci, "pci + pcj"); 
    while(!v.stopped())
    {
      v.runOnce(); 
      usleep(100*1000); 
    }

  }

  ROS_INFO("OK, finish it"); 
 
  return ;

}

void init_parameters()
{
  // parameters configure 
  CParams* pP = CParams::Instance(); 
  pP->m_feature_detector_type = "SIFT";  // SIFTGPU 
  pP->m_feature_descriptor_type = "SIFT";  // SIFTGPU
  pP->m_feature_match_type = "FLANN"; 
  pP->m_nn_distance_ratio = 0.5; // 0.95 for SIFTGPU, 0.5-0.7 for SIFT 
  pP->m_max_dist_for_inliers = 0.05;  
  pP->m_max_num_features = 500; 
  pP->m_ransac_iterations = 5000;
  pP->m_min_matches = 4; 
}
