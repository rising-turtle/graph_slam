
/*
 * Feb. 8, 2017 David Z 
 * 
 * test for VRO and BA using SwissRanger 4000 
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

using namespace cv; 
using namespace std; 
using namespace gtsam; 

#define R2D(r) (((r)*180.)/M_PI)

void init_parameters(); 
void testTwoFrameMatch(); 
void print_tf(ostream& out, tf::Transform tT); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_ba"); 
  ros::NodeHandle n;
  init_parameters(); 
  testTwoFrameMatch(); 
  return 0; 
}

void testTwoFrameMatch()
{
  // read sr4k data from disk 
  string src_file = "../../../src/visual_odometry/data/d1_0726.bdat"; 
  string tar_file = "../../../src/visual_odometry/data/d1_0724.bdat"; 
  
  ros::NodeHandle nh("~"); 
  nh.param("src_filename", src_file, src_file); 
  nh.param("tar_filename", tar_file, tar_file); 

  // CSReader r4k; 
  CSReadCV r4k; 

  // generate imgs and dpts 
  cv::Mat tar_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
  cv::Mat src_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
  cv::Mat src_cv_i_img; 
  cv::Mat tar_cv_i_img; 

  if(!r4k.readOneFrameCV(src_file, src_cv_i_img, src_cv_d_img))
  {
    ROS_INFO("%s failed to read file %s", __FILE__, src_file.c_str()); 
    return ;
  }
  if(!r4k.readOneFrameCV(tar_file, tar_cv_i_img, tar_cv_d_img))
  {
    ROS_INFO("%s failed to read file %s", __FILE__, tar_file.c_str()); 
    return ;
  }

  // VRO 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  sr4k.setDepthScale(0.001); 
  CCameraNode::set_cam_cov(sr4k); 
  CCameraNode::gb_dis_match_point = true; // whether to see the result of 

  // generate node 
  CSparseFeatureVO vo(sr4k); 
  CCameraNodeBA* ni = new CCameraNodeBA(); 
  CCameraNodeBA* nj = new CCameraNodeBA(); 
  vo.featureExtraction(tar_cv_i_img, tar_cv_d_img, 0.001, *ni); 
  vo.featureExtraction(src_cv_i_img, src_cv_d_img, 0.001, *nj); 
  
  MatchingResult mr = nj->matchNodePair(ni); 

  Eigen::Matrix4f Tij = mr.final_trafo; 

  // BA 
  Eigen::Matrix4f Tji = mr.final_trafo.inverse(); 

  map<int, int> match = nj->matchNodePairBA(ni, Tji, &sr4k);
  if(match.size() <= 0)
  {
    ROS_WARN("No match has been found !"); 
  }else
  {
    ROS_WARN("matches %d has been found", match.size()); 
    
    // display the matched point  
    if(CCameraNode::gb_dis_match_point)
    {
      cv::Mat outImg; 
      vector<cv::DMatch> m(match.size()); 
      map<int, int>::iterator it = match.begin(); 
      for(int i=0; i<m.size(); i++)
      {
        DMatch& ma = m[i]; 
        ma.queryIdx = it->second; 
        ma.trainIdx = it->first; 
        ++it; 
      }
      for(int j=0; j<mr.inlier_matches.size(); j++)
      {
        m.push_back(mr.inlier_matches[j]);
      }
      
      cv::drawMatches(nj->m_img, nj->m_feature_loc_2d, ni->m_img, ni->m_feature_loc_2d, m , outImg); 
      cv::imshow("ba_matches", outImg); 
      cv::waitKey(0); 
    }

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

    Pose3 Pi; Pose3 Pj; 
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
    
    isam.update(*g, initialEstimate); 
    isam.update(); 
    
    Values curEstimate = isam.calculateEstimate(); 
    g->resize(0); 
    initialEstimate.clear();

    Pj = curEstimate.at<Pose3>(Symbol('x',1)); 
    
    cout <<" before ba, Tij: "<<endl<<Tij<<endl;

    Eigen::Matrix4d Tba = Pj.matrix(); 
    cout <<" after ba, Tij: "<<endl<<Tba<<endl; 
    Tij = Tba.cast<float>();

  }
  
  

  // print_tf(std::cout, tran); 

  CloudPtr pci(new Cloud); 
  CloudPtr pcj(new Cloud); 
  CloudPtr pcj_ni(new Cloud); 
  generatePointCloud(tar_cv_i_img, tar_cv_d_img, 0.001, sr4k, *pci);
  generatePointCloud(src_cv_i_img, src_cv_d_img, 0.001, sr4k, *pcj); 
  
  pcl::transformPointCloud(*pcj, *pcj_ni,  Tij); // mr.final_trafo); 
  
  markColor(*pci, GREEN); 
  markColor(*pcj_ni, RED); 
  *pci += *pcj_ni; 
  
  CVTKViewer<pcl::PointXYZRGBA> v;
  // v.getViewer()->addCoordinateSystem(0.2, 0, 0); 
  v.addPointCloud(pci, "pci + pcj"); 
  while(!v.stopped())
  {
    v.runOnce(); 
    usleep(100*1000); 
  }

  return ;

}


void print_tf(ostream& out, tf::Transform tT)
{
  tfScalar r, p, y, tx, ty, tz;
  tT.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  out<<"test_vro: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<" qx = "<<
    tT.getRotation().x()<<" qy = "<<tT.getRotation().y()<<" qz= "<<tT.getRotation().z()<<" qw = "<<tT.getRotation().w()<<endl;
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
