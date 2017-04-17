/*
 * Feb. 12, 2017 David Z
 *
 * add vro results at sparse area (SIFT number < Threshold)
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
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include "matching_result.h"
#include "gt_parameter.h"
#include "plane.h"
#include "plane_node.h"

using namespace cv; 
using namespace std; 
using namespace gtsam; 

#define R2D(r) (((r)*180.)/M_PI)

string model = "add"; 
string file = "vro_results.log"; 
string out_file; 
string data_dir = "/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k";
// int min_match_num = 4; 

void init_parameters(); 
void convert_file(); 
bool changeMR(CGraphGT&, MatchingResult*); 

// ofstream g_ouf("matched_number.log"); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "add_vo_at_sparse_area"); 
  ros::NodeHandle n;
  
  ROS_INFO("usage ./add_vo_at_sparse_area vo_file data_dir [suffix] "); 
  if(argc >1 )
    file = argv[1];
  if(argc > 2)
    data_dir = argv[2]; 
  if(argc >3)
    model = argv[3]; 
  // if(argc >4)
    // min_match_num = atoi(argv[4]);

  init_parameters(); 
  convert_file(); 
  return 0; 
}

void convert_file()
{ 
  string g_vro_results_file = file; 
  out_file = g_vro_results_file +"_"+ model;
  CGTParams::Instance()->m_vro_result = out_file; 
    
  // set graph strcuture 
  CGraphGT gt_graph; 

  gt_graph.readVRORecord(g_vro_results_file);

  ofstream * pf = gt_graph.getRecFile(); 

  for(int i=0; i<gt_graph.mv_vro_res.size() && ros::ok(); i++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[i]; 
    // if(pm->edge.id2 != 406 || pm->edge.id1 != 404) continue; 

    if(pm->edge.informationMatrix(0,0) == 10000) 
    {
      changeMR(gt_graph, pm); 
    }

    Pose3 pose(pm->final_trafo.cast<double>()); 
    Vector6 p = Pose3::ChartAtOrigin::Local(pose); 
    (*pf) << pm->edge.id2<<" "<<pm->edge.id1<<" "<<p(0)<<" "<<p(1)<<" "<<p(2) <<" "<<p(3)<<" "<<p(4)<<" "<<p(5)<< " "; 

    for(int i=0; i<6; i++)
    for(int j=i;j<6;j++)
    {
      (*pf)<<pm->edge.informationMatrix(i,j)<<" ";
    }
    (*pf)<<endl;
  }

  return ;
}

bool changeMR(CGraphGT& g, MatchingResult* pm)
{
  // VRO 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  sr4k.setDepthScale(0.001); 
  CCameraNode::set_cam_cov(sr4k); 
  // CCameraNode::gb_dis_match_point = true; // whether to see the result of 

  CSReadCV r4k; 
  cv::Mat ni_rgb, ni_dpt, nj_rgb, nj_dpt; // 
  
  stringstream sni, snj; 
  sni<<data_dir<<"/d1_"<<setfill('0')<<setw(7)<<pm->edge.id1<<".bdat"; 
  snj<<data_dir<<"/d1_"<<setfill('0')<<setw(7)<<pm->edge.id2<<".bdat"; 

  if(!r4k.readOneFrameCV(sni.str(), ni_rgb, ni_dpt))
  {
    ROS_INFO("%s failed to read file %s", __FILE__, sni.str().c_str()); 
    return ;
  }
  if(!r4k.readOneFrameCV(snj.str(), nj_rgb, nj_dpt))
  {
    ROS_INFO("%s failed to read file %s", __FILE__, snj.str().c_str()); 
    return ;
  }

  // generate node 
  CSparseFeatureVO vo(sr4k); 
  CCameraNodeBA* ni = new CCameraNodeBA(); 
  CCameraNodeBA* nj = new CCameraNodeBA(); 
  vo.featureExtraction(ni_rgb, ni_dpt, 0.001, *ni); 
  vo.featureExtraction(nj_rgb, nj_dpt, 0.001, *nj); 
 
  // check whether a plane can be detected, else discard it 
  // generate plane node 
  CPlaneNode* pni = new CPlaneNode(); 
  CPlaneNode* pnj = new CPlaneNode();
  int ri, rj; 

  int ret = pni->extractPlanes(ni_rgb, ni_dpt, &sr4k); 
  ri = ret; 
  if(ret <= 0)
  {
    // ROS_INFO("what? pni has no plane ,idi = %d", idi); 
    return false;
  }else{
    ROS_INFO("pni has %d planes ", pni->mv_planes.size()); 
  }
  
  ret = pnj->extractPlanes(nj_rgb, nj_dpt, &sr4k); 
  rj = ret; 
  if(ret <=0)
  {
    // ROS_INFO("what? pnj has no plane, idj = %d,", idj ); 
    return false; 
  }else{
    ROS_INFO("pnj has %d planes ", pnj->mv_planes.size()); 
  }
  
  // this two planes must be similar
  double COS20 = cos(20.*M_PI/180.); 
  bool good_match = false; 
  {
    for(int i=0; i < ri; i++)
    {
      CPlane* pi = pni->mv_planes[i];  
        for(int j = 0; j<rj; j++)
        {
          CPlane* pj = pnj->mv_planes[j]; 
          double cosA = pi->dotProduct(pj); 
          if(fabs(cosA) >= COS20)
          {
            good_match = true; break; 
          }
        }
      if(good_match) break;  
    }
  }
  
  if(!good_match ) return false;


  // match node through projection vo
  // MatchingResult mr = nj->matchNodePairBA(ni, &sr4k); 
  MatchingResult mr = nj->matchNodePairVRO(ni, &sr4k); 

  if(mr.succeed_match)
  {
    g.computeCovVRO(ni, nj, mr); 
  
    ROS_WARN("Succeed to update edge from %d to %d ", pm->edge.id1, pm->edge.id2);

    pm->final_trafo = mr.final_trafo; 
    pm->edge.informationMatrix = mr.edge.informationMatrix;  
  }else{
    ROS_INFO("failed to update edge from %d to %d", pm->edge.id1, pm->edge.id2); 
  }

  // update pm using ba 
  return true; 
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




