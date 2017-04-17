/*
 * Feb. 14, 2017 David Z 
 * 
 * show registration result in a list 
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

using namespace cv; 
using namespace std; 
using namespace gtsam; 

string g_file_dir; 
string g_vro_results_file; 


#define R2D(r) (((r)*180.)/M_PI)

void showResultInList(); 
void testTwoFrameMatch(CGraphGT& , int i, int j);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "view_registration_list"); 
  ros::NodeHandle n;
  showResultInList(); 
  return 0; 
}

void showResultInList()
{
  string vo_list; 
  ros::NodeHandle nh("~");
  int min_num = 0;
  int max_num = 20; 
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k")); // default parameters
  // nh.param("vro_results_file", g_vro_results_file, string("vro_results.log_ba_5")); 
  // nh.param("vro_results_file", g_vro_results_file, string("vo_results_merge.log")); 
  nh.param("vro_results_file", g_vro_results_file, string("vro_results.log_hybrid_ba_8")); 
  nh.param("vo_list_file", vo_list, string("retained_vo_edge_list.log")); 
  nh.param("min_number", min_num, min_num); 
  nh.param("max_number", max_num, max_num); 

   // set graph strcuture 
  CGraphGT gt_graph; 
   // VRO result 
  gt_graph.readVRORecord(g_vro_results_file); 
 
  int idi, idj, n; 
  double e, er; 
  ifstream inf(vo_list.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open file : %s", vo_list.c_str()); 
    return ;
  }

  while(!inf.eof() && ros::ok())
  {
    inf >> idj >> idi >> n >> e >> er ; // >> e >> er; 
    if(inf.eof()) break; 
    ROS_INFO("view registration %d and %d, n = %d e = %lf", idj, idi, n, e);
    if(n >= min_num && n<=max_num) // 
    {
      testTwoFrameMatch(gt_graph, idi, idj); 
    }
  }

}

void testTwoFrameMatch( CGraphGT& gt_graph, int idi, int idj)
{
    // VRO 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  sr4k.setDepthScale(0.001); 
  CCameraNode::set_cam_cov(sr4k); 
  CCameraNode::gb_dis_match_point = true; // whether to see the result of

  for(int k=0; k<gt_graph.mv_vro_res.size() && ros::ok(); k++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[k]; 
    if( pm->edge.id2 != idj || pm->edge.id1 != idi) continue; 
      
    if(pm->edge.informationMatrix(0,0) == 10000)
    {
      ROS_INFO("Edge %d and %d is void", idj, idi);
      continue; 
    }

    Matrix4 TMij = pm->final_trafo.cast<double>(); 
    // getframe 
    stringstream sni, snj; 
    sni << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<idi<<".bdat"; 
    snj << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<idj<<".bdat"; 
    
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
    break; 
  }

 
  return ;

}


