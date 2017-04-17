/*
 * Feb. 26, 2017 David Z
 *
 * convert the result of vro to ba, using RANSAC find inliers, and run ba next 
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

string model = "ba"; 
string file = "vro_results.log"; 
string out_file; 
string data_dir = "/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k";
int min_match_num = 20; 

void init_parameters(); 
void convert_file(); 
void print_tf(ostream& out, tf::Transform tT); 
void makeItVoid(MatchingResult*); 
bool changeMR(MatchingResult*); 
bool changeMRByVRO(MatchingResult*); 

bool bundleAdjust(MatchingResult& , CCameraNodeBA* ni, CCameraNodeBA* nj, CamModel*); 

ofstream* g_ouf = 0; // ("matched_number.log"); 
CGraphGT* pg= NULL ; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_ba_2"); 
  ros::NodeHandle n;
  
  ROS_INFO("usage ./convert_vo2ba vo_file data_dir [model] [N] "); 
  ROS_INFO(" vo_file : the file from vro "); 
  ROS_INFO(" data_dir: camera data dir "); 
  // ROS_INFO(" model: na or hybrid"); 
  ROS_INFO(" N: minimal number of features to match");

  g_ouf = new ofstream("matched_number.log"); 
  if(!g_ouf->is_open())
  {
    ROS_ERROR("failed to open file matched_number.log");
    return -1; 
  }

  if(argc >1 )
    file = argv[1];
  if(argc > 2)
    data_dir = argv[2]; 
  if(argc >3)
    min_match_num = atoi(argv[3]);

  pg = new CGraphGT; 

  init_parameters(); 
  convert_file(); 
  g_ouf->flush(); 
  g_ouf->close(); 
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
    ROS_INFO("Let's handle edge from %d to %d in model %s", pm->edge.id1, pm->edge.id2, model.c_str());

    if(!changeMR(pm)) // failed to change it 
    {
      if(pm->edge.informationMatrix(0,0) != 10000) // make it as failed 
      {
        makeItVoid(pm);     
      }
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

bool changeMR(MatchingResult* pm)
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
    return false;
  }
  if(!r4k.readOneFrameCV(snj.str(), nj_rgb, nj_dpt))
  {
    ROS_INFO("%s failed to read file %s", __FILE__, snj.str().c_str()); 
    return false;
  }

  // generate node 
  CSparseFeatureVO vo(sr4k); 
  CCameraNodeBA* ni = new CCameraNodeBA(); 
  CCameraNodeBA* nj = new CCameraNodeBA(); 
  vo.featureExtraction(ni_rgb, ni_dpt, 0.001, *ni); 
  vo.featureExtraction(nj_rgb, nj_dpt, 0.001, *nj); 
 
  // 
  MatchingResult tmp ; 
  tmp.edge.id1 = pm->edge.id1; 
  tmp.edge.id2 = pm->edge.id2; 
  tmp.final_trafo = pm->final_trafo; 
  tmp.edge.informationMatrix = pm->edge.informationMatrix; 

  // update pm using ba 
  // bool succeed = bundleAdjust(*pm, ni, nj, &sr4k); 
  bool succeed = bundleAdjust(tmp, ni, nj, &sr4k); 
  if(!succeed) 
  {
    delete ni; 
    delete nj; 
    return false; 
  }

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity(); 

  // if(MatrixEqual(pm->final_trafo, I, 1e-4))
  if(MatrixEqual(tmp.final_trafo, I, 1e-4))
  {
    // makeItVoid(pm); 
  }else
  {
    pm->final_trafo = tmp.final_trafo; 
    pm->edge.informationMatrix = tmp.edge.informationMatrix; 
  }

  delete ni; 
  delete nj;
  return true; 
}

bool bundleAdjust(MatchingResult& mr, CCameraNodeBA* ni, CCameraNodeBA* nj, CamModel* pcam)
{
  // BA 
  Eigen::Matrix4f Tji = mr.final_trafo.inverse(); 
  map<int, int> match = nj->matchNodePairBA(ni, Tji, pcam); // given Transformation from VRO, the inliers can be easily obtained 

  bool ret = true; 
  // find inliers 
  if(mr.edge.informationMatrix(0,0) == 10000) // VRO failed on this match, need to run RANSAC to find out inliers 
  {
    // map<int, int> match = nj->matchNodePairInliers(ni, Tji, pcam); 
    match = nj->matchNodePairInliers(ni, Tji, pcam); 
  }
  (*g_ouf)<<mr.edge.id2<<" "<<mr.edge.id1<<" "<<match.size()<<endl;

  if(match.size() < min_match_num)
  {
    ROS_WARN("No match has been found !");  
    return false;
  }else
  {
    ROS_WARN("matches %d has been found", match.size()); 

    ISAM2Params param; 
    param.relinearizeThreshold = 0.01; 
    param.relinearizeSkip = 1;
    ISAM2 isam(param); 

    NonlinearFactorGraph* g = new NonlinearFactorGraph; 
    // pr
    Vector6 s; // s << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3;
    s << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(s);
    noiseModel::Isotropic::shared_ptr measNoise = noiseModel::Isotropic::Sigma(2, 1.);

    Pose3 Pi;  
    // Pose3 Pij(mr.final_trafo.cast<double>()); 
    Pose3 Pj; // = Pi * Pij; 
    // Cal3_S2::shared_ptr K(new Cal3_S2(250.5773, 250.5773, 0, 90, 70));

    boost::shared_ptr<Cal3DS2> K(new Cal3DS2(250.5773, 250.5773, 0, 90, 70, -0.8466, 0.5370));

    g->push_back(PriorFactor<Pose3>(Symbol('x', 0), Pi, priorNoise)); 
    map<int, int>::iterator it = match.begin(); 
    Values initialEstimate ; // = new Values; 

    ROS_INFO("%d before insert into factor graph", __LINE__); 

    // insert pose 
    // noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
    initialEstimate.insert<Pose3>(Symbol('x',0), Pi);
    initialEstimate.insert<Pose3>(Symbol('x',1), Pj); 
    int j = 0;
    while(it != match.end())
    {
      Eigen::Vector4f& pt = ni->m_feature_loc_3d[it->first]; 
      Point3 q ; q << pt(0), pt(1), pt(2); 

      initialEstimate.insert<Point3>(Symbol('l',j), q);
      Point2 measure_ni;
      Point2 measure_nj;
   
      measure_ni(0) = ni->m_feature_loc_2d[it->first].pt.x; 
      measure_ni(1) = ni->m_feature_loc_2d[it->first].pt.y; 

      measure_nj(0) = nj->m_feature_loc_2d[it->second].pt.x; 
      measure_nj(1) = nj->m_feature_loc_2d[it->second].pt.y; 
      
      // if(j == 0)
      {
        noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.014); 
        g->push_back(PriorFactor<Point3>(Symbol('l',j), q, pointNoise)); 
      }

      // ROS_INFO("%d before add measurement factor ", __LINE__);
      g->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_ni, measNoise, Symbol('x',0), Symbol('l', j), K)); 
      g->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_nj, measNoise, Symbol('x',1), Symbol('l', j), K)); 
      
      ++j; 
      ++it; 
      // ROS_INFO("%d add visual feature %d", __LINE__, j);
    }
    
    // isam.update(*g, initialEstimate); 
    // isam.update(); 
    
    LevenbergMarquardtOptimizer optimizer(*g, initialEstimate); 
    Values curEstimate; 
      try{
      cout <<"before optimization \n";
      curEstimate = optimizer.optimize(); 
      cout <<"after optimization \n"; 

    // Values curEstimate = isam.calculateEstimate(); 
    Pj = curEstimate.at<Pose3>(Symbol('x',1)); 
    
    // cout <<" before ba, Tij: "<<endl<<Tij<<endl;

    // Eigen::Matrix4d Tba = Pj.matrix(); 
    // cout <<" after ba, Tij: "<<endl<<Tba<<endl; 
    // Tij = Tba.cast<float>();
      mr.final_trafo = Pj.matrix().cast<float>(); 
      Marginals marginals(*g, curEstimate, Marginals::CHOLESKY); 
      cout <<"after marginalization\n";

      Matrix6 S_pose = marginals.marginalCovariance(Symbol('x', 1)); 
      mr.edge.informationMatrix = S_pose.inverse();  

    }catch(...)
    {
      cerr <<" convert_vo2ba_2.cpp: something error here "<<__LINE__<<" return false\n"; 
      ret = false; 
    }

    g->resize(0); 
    initialEstimate.clear();
    delete g; 
  }
    return ret; 
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



