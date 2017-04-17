/*
 * Feb. 14, 2017 David Z
 *
 * merge the ba results with those from vro when the number of features is below a threshold 
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
string match_file("input_match.log");

string out_file("vo_results_merge.log"); 

void mergeTwoVO(); 

bool findMatch(CGraphGT& g, int idi, int idj, Eigen::Matrix4f&); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "merge_vo_results"); 
  ros::NodeHandle n;

  ros::NodeHandle nh("~"); 
  nh.param("vro_results_file1", file_1, string("vro_results.log_ba_5")); 
  nh.param("vro_results_file2", file_2, string("vro_results.log")); 

  if(argc > 1) 
    file_1 = argv[1];
  if(argc > 2)
    file_2 = argv[2]; 

  ROS_INFO("merge %s with %s into %s", file_1.c_str(), file_2.c_str(), out_file.c_str()); 

  mergeTwoVO(); 
  return 0; 
}

void mergeTwoVO()
{
  CGraphGT g1; 
  CGraphGT g2; 

  g1.readVRORecord(file_1);  // file_1 ba result 
  g2.readVRORecord(file_2);  // file_2 vro result 
    
  ifstream inf(match_file.c_str()); 

  int idi, idj, n; 
  for(int i=0; i<g1.mv_vro_res.size() && ros::ok(); i++)
  {
      MatchingResult* pm = g1.mv_vro_res[i];  

      inf >> idj >> idi >> n; 

      if(idj != pm->edge.id2 || idi != pm->edge.id1) 
      {
        ROS_ERROR("idj != pm->edge.id2 something error in merge_vro.cpp"); 
        break; 
      }
      
      MatchingResult * pm2 = g2.mv_vro_res[i]; 
      
      if(idj != pm2->edge.id2 || idi != pm2->edge.id1) 
      {
        ROS_ERROR("idj != pm2->edge.id2 something error in merge_vro.cpp"); 
        break; 
      }
      
      // let's see whether need to merge 
      if(n >= 20) // no need to worry 
      {
      
      
      }else
      {
        if(pm2->edge.informationMatrix(0,0) != 10000)
        {
          // replaced by vro's results 
          // pm->edge.transformation = pm2->edge.transformation; 
          pm->final_trafo = pm2->final_trafo; 
          pm->edge.informationMatrix =  pm2->edge.informationMatrix; 
        }
      }

  }

  // 2, save 
  ofstream * pf = new ofstream(out_file.c_str()); 

  for(int i=0; i<g1.mv_vro_res.size(); i++)
  {
    MatchingResult* pm = g1.mv_vro_res[i]; 

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


