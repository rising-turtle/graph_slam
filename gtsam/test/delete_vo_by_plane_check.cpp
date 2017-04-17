/*
 * Feb. 11, 2017 David Z
 *
 * delete the results of vro that conflicts with plane check 
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

#define R2D(r) (((r)*180.)/M_PI)

// string file = "vro_results.log_hybrid"; 
// string file = "vro_results.log_ba_5"; 
// string file = "vo_results_merge.log"; 
string file = "vro_results.log_hybrid"; 
string model = "final";
string num_err_file ="num_vs_err_new.log";
string out_file; 
int min_match_num = 20; // 25
double min_m_dis = 6.25; 
double min_dis_T2 = 0.584; 

// void init_parameters(); 
void convert_file(); 
void print_tf(ostream& out, tf::Transform tT); 
void makeItVoid(MatchingResult*); 

void devoidVO(CGraphGT&, int idj, int idi); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "refine_vo"); 
  ros::NodeHandle n;

  if(argc > 1) 
    file = argv[1];
  
  convert_file(); 
  return 0; 
}

void convert_file()
{ 
  string g_vro_results_file = file; 
  out_file = g_vro_results_file +"_"+ model;
    
  // set graph strcuture 
  CGraphGT gt_graph; 
  gt_graph.readVRORecord(g_vro_results_file);

  ifstream inf(num_err_file.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open file %s", num_err_file.c_str()); 
    return ;
  }
  
  // 1, change 
  int idi, idj, n; 
  double e, er, ne, ner; 
  ofstream ouf("retained_vo_edge_list.log");
  ofstream ouf2("deleted_vo_edge_list.log");
  while(!inf.eof())
  {
    // inf >> idj >> idi >> n >> e >> er >> ne >> ner ; 
    inf >> idj >> idi >> n >> ne >> ner; 
    if(inf.eof()) break; 
    
    if(n > min_match_num) { 
      // ROS_INFO("from idj %d to idi %d n = %d continue", idj, idi, n); 
      continue; // skip large number matches  
    }
    
    if(n < 4) continue ;

    if(ne >= min_m_dis)
    {
      ROS_ERROR("idj : %d idi : %d n : %d ne : %lf get rid of it!", idj, idi, n, ne); 
      devoidVO(gt_graph, idj, idi); 
      ouf2 <<idj<<"\t"<<idi<<"\t"<<n<<"\t"<<ne<<"\t"<<ner<<endl;
    }else{
      
      if(ne == 0)
      {
        if(n < 12)
        {
          devoidVO(gt_graph, idj, idi); 
          ouf2 <<idj<<"\t"<<idi<<"\t"<<n<<"\t"<<ne<<"\t"<<ner<<endl;
        }else
        {
          ouf <<idj<<"\t"<<idi<<"\t"<<n<<"\t"<<ne<<"\t"<<ner<<endl;
        }
      }else
        ouf <<idj<<"\t"<<idi<<"\t"<<n<<"\t"<<ne<<"\t"<<ner<<endl;
    }

    /*
    if(n >= 12)
    {
      if(ne >= min_m_dis)
      {
          ROS_ERROR("idj : %d idi : %d n : %d ne : %lf get rid of it!", idj, idi, n, ne); 
          devoidVO(gt_graph, idj, idi); 
      }else{
        ouf <<idj<<"\t"<<idi<<"\t"<<n<<"\t"<<ne<<"\t"<<ner<<endl;
      }
    }else{
      if(ne >= min_dis_T2)
      {
          ROS_ERROR("idj : %d idi : %d n : %d ne : %lf get rid of it!", idj, idi, n, ne); 
          devoidVO(gt_graph, idj, idi); 
      }else{
          ouf <<idj<<"\t"<<idi<<"\t"<<n<<"\t"<<ne<<"\t"<<ner<<endl;
      }
    }*/
  }

  // 2, save 
  ofstream * pf = new ofstream(out_file.c_str()); 

  for(int i=0; i<gt_graph.mv_vro_res.size(); i++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[i]; 

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


void devoidVO(CGraphGT& gt_graph, int idj, int idi)
{
  for(int i=0; i<gt_graph.mv_vro_res.size(); i++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[i]; 
    if(pm->edge.id2 != idj || pm->edge.id1 != idi) continue; 
    makeItVoid(pm); 
      return; 
  }

}

void makeItVoid(MatchingResult* pm)
{
  pm->final_trafo = Eigen::Matrix4f::Identity(); 
  for(int i=0; i<6; i++)
    for(int j=i;j<6;j++)
    {
      if(j==i)
        pm->edge.informationMatrix(i,i) = 10000;
      else
        pm->edge.informationMatrix(i,j) = 0;
    }
  return ; 
}



