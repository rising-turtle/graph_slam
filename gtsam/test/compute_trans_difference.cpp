/*
 * Feb. 14, 2017 David Z
 *
 * compute the transformation differences between 
 * two vo results
 *
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

using namespace cv; 
using namespace std; 
using namespace gtsam; 

string file_1; 
string file_2; 

string out_file("vo_results_diff.log"); 

void computeDiff(); 

bool findMatch(CGraphGT& g, int idi, int idj, Eigen::Matrix4f&); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "compute_vo_difference"); 
  ros::NodeHandle n;

  ros::NodeHandle nh("~"); 
  nh.param("vro_results_file1", file_1, string("vro_results.log_hybrid")); 
  nh.param("vro_results_file2", file_2, string("vro_results.log_hybrid_final")); 
  // nh.param("vro_results_file1", file_1, string("vro_results.log_ba_5_final")); 
  // nh.param("vro_results_file2", file_2, string("vro_results.log_hybrid_add_final")); 



  if(argc > 1) 
    file_1 = argv[1];
  if(argc > 2)
    file_2 = argv[2]; 
  computeDiff(); 
  return 0; 
}

void computeDiff()
{
  CGraphGT g1; 
  CGraphGT g2; 

  g1.readVRORecord(file_1); 
  g2.readVRORecord(file_2); 

  ofstream ouf(out_file.c_str()); 

  for(int i=0; i<g1.mv_vro_res.size() && ros::ok(); i++)
  {
      MatchingResult* pm = g1.mv_vro_res[i]; 
      Eigen::Matrix4f Tij1 = pm->final_trafo; 
      Eigen::Matrix4f Tij2; 
      Pose3 PI = Pose3::identity(); 
      if(findMatch(g2, pm->edge.id1, pm->edge.id2, Tij2))
      {
        Eigen::Matrix4f dT = Tij1*Tij2.inverse(); 
        Pose3 dp(dT.cast<double>()); 
        Rot3 dR = dp.rotation(); 
        Vector3 dt = dp.translation(); 
        
        Pose3 P1(Tij1.cast<double>()); 
        Pose3 P2(Tij2.cast<double>()); 

        if(dp.equals(PI, 1e-6)){
        ouf <<pm->edge.id2<<"\t"<<pm->edge.id1<<"\t 0 0 0 0 0 0 ";
        }else{
        ouf << pm->edge.id2<<"\t"<<pm->edge.id1<<"\t"<<dt(0)<<" "<<dt(1)<<" "<<dt(2)<<" "<<R2D(dR.roll())<<" "<<R2D(dR.pitch())<<" "<<R2D(dR.yaw());
        }
        
        dt = P1.translation(); 
        dR = P1.rotation(); 
        Vector3 rpy = dR.rpy();
        ouf << dt.norm() <<" "<< rpy.norm()<<endl;
        

      }else{
      
        ouf <<pm->edge.id2<<"\t"<<pm->edge.id1<<"\t -1 -1 -1 -1 -1 -1"<<endl; 
      }
  }
  return ;
}


bool findMatch(CGraphGT& g, int idi, int idj, Eigen::Matrix4f& m)
{
    for(int i=0; i<g.mv_vro_res.size() && ros::ok(); i++)
  {
    MatchingResult* pm = g.mv_vro_res[i]; 
    if(pm->edge.id1 == idi && pm->edge.id2 == idj) 
    {
      m = pm->final_trafo; 
      return true; 
    }
  }
  return false;
}






