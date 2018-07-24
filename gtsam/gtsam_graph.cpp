
/*
 * Oct. 3, 2016, David Z 
 * 
 * Graph interface with gtsam 
 *
 * */

#include "gtsam_graph.h"
#include <ros/ros.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>
#include <QList>
#include <QThread>
#include <QtConcurrentMap>
#include <qtconcurrentrun.h>
#include "matching_result.h"
#include "camera_node.h"
#include "camera_node_ba.h"
#include "gt_parameter.h"
#include "opencv2/opencv.hpp"
#include "plane.h"
#include "plane_node.h"
#include "cam_model.h"
#include "chi2.h"
#include "transformation_estimation_euclidean.h"

using namespace gtsam;
using namespace std; 

#define INVALID_NUM 111

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 
using symbol_shorthand::L; // Plane landmark (nv, d)
using symbol_shorthand::Q; // Point3 (x, y, z)

Eigen::Matrix<double, 6, 1> cov_Helper(Eigen::Matrix4f& m)
{
  Eigen::Matrix4d md = m.cast<double>(); 
  Pose3 p(md); 
  gtsam::Vector6 r = Pose3::ChartAtOrigin::Local(p); 
  return r; 
} 

ofstream* CGraphGT::getRecFile()
{
  static ofstream * pf = 0 ;
  if(pf == 0)
  {
    // pf = new ofstream("vro_results.log"); 
    pf = new ofstream(CGTParams::Instance()->m_vro_result.c_str());
  }
  return pf; 
}

CGraphGT::CGraphGT()
{
  mp_fac_graph = new NonlinearFactorGraph; 
  mp_new_fac = new NonlinearFactorGraph; 
  mp_node_values = new Values ;
  mp_new_node = new Values; 

  initISAM2Params(); 
  mp_w2o = new Pose3;
  mp_u2c = new Pose3; 
  mp_prev_bias = new imuBias::ConstantBias; 
  mp_prev_state = new NavState;
  // mb_record_vro_results = true; // TODO: parameterize this one  
  mb_record_vro_results = CGTParams::Instance()->m_record_vro_results; 
  m_plane_landmark_id = 0; 
  m_sift_landmark_id =0 ;
}

void CGraphGT::initISAM2Params()
{
  mp_isam2_param = new ISAM2Params; 
  mp_isam2_param->relinearizeThreshold = 0.1; // 0.3 0.2
  mp_isam2_param->relinearizeSkip = 1; // 2
  mp_isam2 = new ISAM2(*mp_isam2_param); 
}

CGraphGT::~CGraphGT(){
  
  if(mp_prev_bias != NULL) 
  {
    delete mp_prev_bias; mp_prev_bias = NULL;
  }
  if(mp_prev_state != NULL)
  {
    delete mp_prev_state; mp_prev_state = NULL; 
  }

  if(mp_fac_graph != NULL) 
  {
    delete mp_fac_graph;  mp_fac_graph = NULL; 
  }
  
  if(mp_new_fac != NULL) 
  {
    delete mp_new_fac; mp_new_fac = NULL; 
  }

  if(mp_new_node != NULL)
  {
    delete mp_new_node; mp_new_node = NULL; 
  }

  if(mp_node_values != NULL)
  {
    delete mp_node_values; mp_node_values = NULL; 
  }

  if(mp_w2o != NULL)
  {
    delete mp_w2o;  mp_w2o = NULL; 
  }
  
  if(mp_u2c != NULL)
  {
    delete mp_u2c;  mp_u2c = NULL; 
  }

  if(mp_isam2_param != NULL)
  {
    delete mp_isam2_param; mp_isam2_param = NULL; 
  }

  if(mp_isam2 != NULL)
  {
    delete mp_isam2; mp_isam2 = NULL; 
  }

  for(map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    delete it->second; 
  }

}

void CGraphGT::writeGTSAM(string f)
{
  ofstream ouf(f); 
  if(!ouf.is_open())
  {
    cerr<<__FILE__<<": failed to open file "<<f<<" to writeGTSAM structure"<<endl; 
    return ; 
  }
  
  mp_fac_graph->saveGraph(ouf, *mp_node_values); 
  
}

double CGraphGT::error()
{
  return mp_fac_graph->error(*mp_node_values);
}

void CGraphGT::setWorld2Original(double p)
{
  //// WORLD Coordinate System 
  //          
  //          Z   X
  //          |  /
  //          | /
  //     Y____|/
  //        
  //   CAMERA Coordinate System    
  //           Z 
  //          /
  //         / 
  //        /  
  //        ------- X
  //        |
  //        |
  //        |Y
  //
  //
  Rot3 R_g2b = Rot3::RzRyRx(-M_PI/2., 0, -M_PI/2.); // roll, pitch, yaw, in WORLD coordinate system
  Rot3 R_b2o = Rot3::RzRyRx(p ,0 , 0); 
  Rot3 R_g2o = R_g2b * R_b2o; 
  // Rot3 R_g2o = R_b2o * R_g2b; 

  Point3 t = Point3::Zero(); 
  (*mp_w2o) = Pose3::Create(R_g2o, t);
   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<(*mp_w2o)*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 
}

void CGraphGT::setCamera2IMUTranslation(double px, double py, double pz) // only set translation 
{
    Rot3 R_g2o;  
    Point3 t(px, py, pz); 
    (*mp_u2c) = Pose3::Create(R_g2o, t);
}


void CGraphGT::setCamera2IMU(double p)
{
  //// Body/IMU Coordinate System 
  //          
  //             X
  //            /
  //           /
  //          /_____ Y
  //          |
  //          |
  //        Z | 
  //
  //        
  //   CAMERA Coordinate System    
  //           Z 
  //          /
  //         / 
  //        /  
  //        ------- X
  //        |
  //        |
  //        |Y
  //

  Rot3 R_g2b = Rot3::RzRyRx( M_PI/2., 0. , M_PI/2.); // roll, pitch, yaw, in BODY coordinate system 
  Rot3 R_b2o = Rot3::RzRyRx(p ,0 , 0); 
  Rot3 R_g2o = R_g2b * R_b2o; 
  // Rot3 R_g2o = R_b2o * R_g2b; 

  Point3 t = Point3::Zero(); 
  (*mp_u2c) = Pose3::Create(R_g2o, t);
   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<(*mp_u2c)*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 
}

void CGraphGT::computeCovVRO(CCameraNode* pOld, CCameraNode* pNew, MatchingResult& m)
{
  Eigen::Matrix<double, 6, 6> cov; 
  pNew->computeCov(pOld, m.inlier_matches, cov_Helper, cov); 
  m.edge.informationMatrix = cov.inverse(); 
  if(std::isnan(m.edge.informationMatrix(0,0)))
  {
    cout<<"what? cov: \n"<<cov<<"\ncov.inverse():\n"<<cov.inverse()<<endl;
  }
}

void CGraphGT::computeCovVRO(CCameraNode* pNew, MatchingResult& m)
{
  CCameraNode* pOld = m_graph_map[m.edge.id1]; 
  Eigen::Matrix<double, 6, 6> cov; 
  pNew->computeCov(pOld, m.inlier_matches, cov_Helper, cov); 
  m.edge.informationMatrix = cov.inverse(); 
  if(std::isnan(m.edge.informationMatrix(0,0)))
  {
    cout<<"what? cov: \n"<<cov<<"\ncov.inverse():\n"<<cov.inverse()<<endl;
  }
}

// input ax, ay, az represents normalized gravity vector 
// at initial phase where the imu is assumed static 
void CGraphGT::initFromImu(double ax, double ay, double az)
{
    if(m_graph_map.size() <= 0)
    {
	cerr<<"No node has been created before calling initFromImu!"<<endl; 
	return ; 
    }
    
    // compute rotation for the first pose 
    Eigen::Vector3d fv(ax, ay, az); 
    Eigen::Vector3d tv(0, 0, 1);  // vn100's gz points to upwards
    Eigen::Vector3d w = fv.cross(tv).normalized(); 
    double angle = acos(fv.dot(tv)); 
    
    double half_angle = angle /2.;
    Eigen::Vector4d vq; 
    vq.head<3>() = w * sin(half_angle); 
    vq[3] = cos(half_angle); 
    Eigen::Quaterniond q(vq); 
    Eigen::Matrix<double, 3, 3> m = q.toRotationMatrix(); 
    
    // cout <<"fv= "<<endl<<fv<<endl;
    // cout <<"m = "<<endl<<m<<endl; 
    
    Eigen::Vector3d dst_t = m * fv; 
    // cout <<"dst_t: "<<dst_t.normalized()<<endl; 

    Rot3 R(m);
    Point3 t(0, 0, 0); 
    Pose3 new_pose(R,t); 
    
    *(mp_w2o) = new_pose; 

    // new_pose.print("new_pose");
    // mp_node_values->update(X(m_graph_map[0]->m_id), new_pose); 
    // mp_new_node->update(X(m_graph_map[0]->m_id), new_pose); 
    return ; 
}

void CGraphGT::firstNode(CCameraNode* n, bool online)
{
    // 1, ids 
    n->m_id = m_graph_map.size();
    m_sequence_id = 0; 
    if(online)
    {
      n->m_seq_id = ++ m_sequence_id; 
    }

    // 2, first Pose and add into graph 
    Pose3 origin_priorMean(Eigen::Matrix4d::Identity());
    mp_node_values->insert(X(n->m_id), origin_priorMean);
    // for isam2
    mp_new_node->insert(X(n->m_id), origin_priorMean);

    ROS_INFO("insert X(%d)", n->m_id); 

    Vector6 s; // s << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3;
    s << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(s);
    mp_fac_graph->add(PriorFactor<Pose3>(X(n->m_id), origin_priorMean, priorNoise));

    // for isam2
    mp_new_fac->add(PriorFactor<Pose3>(X(n->m_id), origin_priorMean, priorNoise));

    m_graph_map[n->m_id] = n; 

    // 3, imu part 
    Vector3 priorVelocity; priorVelocity << 0, 0, 0; 
    mp_node_values->insert(V(n->m_id), priorVelocity); 
    mp_new_node->insert(V(n->m_id), priorVelocity); 

    imuBias::ConstantBias priorBias; 
    mp_node_values->insert(B(n->m_id), priorBias); 
    mp_new_node->insert(B(n->m_id), priorBias); 
    
    // Assemble prior noise model and add it in the graph 
    // noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad, rad, rad, m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-3); // m/s 
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);  

   mp_fac_graph->add(PriorFactor<Vector3>(V(n->m_id), priorVelocity, velocity_noise_model)); 
   mp_fac_graph->add(PriorFactor<imuBias::ConstantBias>(B(n->m_id), priorBias, bias_noise_model));
   
   // for isam2 
   mp_new_fac->add(PriorFactor<Vector3>(V(n->m_id), priorVelocity, velocity_noise_model)); 
   mp_new_fac->add(PriorFactor<imuBias::ConstantBias>(B(n->m_id), priorBias, bias_noise_model));
}

bool CGraphGT::addToGTSAM(CCameraNodeBA* ni, CCameraNodeBA* nj, map<int, int>& matches, CamModel* pcam)
{
  map<int, int>::iterator it = matches.begin(); 
  boost::shared_ptr<Cal3DS2> K(new Cal3DS2(pcam->fx, pcam->fy, 0, pcam->cx, pcam->cy, pcam->k1, pcam->k2));

  // ROS_INFO("here 1?");
  Pose3 Pi = mp_node_values->at<Pose3>(X(ni->m_id)); 
  Pose3 Pj = mp_node_values->at<Pose3>(X(nj->m_id)); 

  noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.014); 
  noiseModel::Isotropic::shared_ptr measNoise = noiseModel::Isotropic::Sigma(2, 1.);

  while(it != matches.end())
  {
    if(ni->mv_feature_qid[it->first] == -1 && nj->mv_feature_qid[it->second] == -1) // a total new feature point 
    {
        // add a new sift landmark 
        Eigen::Vector4f& pt = ni->m_feature_loc_3d[it->first]; 
        Point3 q; q << pt(0), pt(1), pt(2); 
        q = mp_u2c->transform_from(q); 
        q = Pi.transform_from(q); 
        mp_node_values->insert<Point3>(Q(m_sift_landmark_id), q); 
        mp_new_node->insert<Point3>(Q(m_sift_landmark_id), q); 
        
        mp_fac_graph->push_back(PriorFactor<Point3>(Q(m_sift_landmark_id), q, pointNoise)); 
        mp_new_fac->push_back(PriorFactor<Point3>(Q(m_sift_landmark_id), q, pointNoise));

        // add two factors 
        Point2 measure_ni, measure_nj; 
        measure_ni(0) = ni->m_feature_loc_2d[it->first].pt.x; 
        measure_ni(1) = ni->m_feature_loc_2d[it->first].pt.y; 

        measure_nj(0) = nj->m_feature_loc_2d[it->second].pt.x; 
        measure_nj(1) = nj->m_feature_loc_2d[it->second].pt.y; 
        
        mp_fac_graph->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_ni, measNoise, X(ni->m_id), Q(m_sift_landmark_id), K, 0, 0, *mp_u2c)); 
        mp_fac_graph->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_nj, measNoise, X(nj->m_id), Q(m_sift_landmark_id), K, 0, 0, *mp_u2c)); 
 
        mp_new_fac->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_ni, measNoise, X(ni->m_id), Q(m_sift_landmark_id), K, 0, 0, *mp_u2c)); 
        mp_new_fac->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_nj, measNoise, X(nj->m_id), Q(m_sift_landmark_id), K, 0, 0, *mp_u2c)); 
 
        ni->mv_feature_qid[it->first] = m_sift_landmark_id; 
        nj->mv_feature_qid[it->second] = m_sift_landmark_id; 
        m_sift_landmark_id++; 

    }else if(ni->mv_feature_qid[it->first] == -1)
    {
      // add one factor between Xi and Q
      int sift_id = nj->mv_feature_qid[it->second]; 
      Point2 measure_ni; 
      measure_ni(0) = ni->m_feature_loc_2d[it->first].pt.x; 
      measure_ni(1) = ni->m_feature_loc_2d[it->first].pt.y; 

      mp_fac_graph->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_ni, measNoise, X(ni->m_id), Q(sift_id), K, 0, 0, *mp_u2c)); 
      mp_new_fac->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_ni, measNoise, X(ni->m_id), Q(sift_id), K, 0, 0, *mp_u2c)); 
      ni->mv_feature_qid[it->first] = sift_id; 
    
    }else if(nj->mv_feature_qid[it->second] == -1)
    {
      int sift_id = ni->mv_feature_qid[it->first] ;
      Point2 measure_nj; 
      measure_nj(0) = nj->m_feature_loc_2d[it->second].pt.x; 
      measure_nj(1) = nj->m_feature_loc_2d[it->second].pt.y; 
      mp_fac_graph->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_nj, measNoise, X(nj->m_id), Q(sift_id), K, 0, 0, *mp_u2c)); 
      mp_new_fac->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_nj, measNoise, X(nj->m_id), Q(sift_id), K, 0, 0, *mp_u2c)); 
      
      nj->mv_feature_qid[it->second] = sift_id; 

    }else{ 
      if(nj->mv_feature_qid[it->second] != ni->mv_feature_qid[it->first])
      {
        ROS_ERROR("%s what? Line %d different landmark at the same position", __FILE__, __LINE__); 
      }
    }
    ++it; 
  }

  return true; 
}

bool CGraphGT::vroAdjust(MatchingResult* pm, CCameraNode* pNewNode, CamModel* pcam)
{
    int pre_id1 = pm->edge.id1; 
    int pre_id2 = pm->edge.id2; 
    correctMatchingID(pm); 
    
    CCameraNodeBA* ni = dynamic_cast<CCameraNodeBA*>(m_graph_map[pm->edge.id1]); 
    // CCameraNodeBA* nj = dynamic_cast<CCameraNodeBA*>(m_graph_map[pm->edge.id2]); 
    CCameraNodeBA* nj = dynamic_cast<CCameraNodeBA*>(pNewNode); 

    Eigen::Matrix4f Tji = pm->final_trafo.inverse(); 
    map<int, int> matches = nj->matchNodePairBA(ni, Tji, pcam); 
    
    if(matches.size() <= 4) 
    {
      if(pm->edge.informationMatrix(0,0) == 10000)
      {
        ROS_ERROR("Nothing changed for edge from %d to %d", pre_id1, pre_id2); 
      }
      pm->edge.id1 = pre_id1; 
      pm->edge.id2 = pre_id2;
      return false; 
    }

    ROS_INFO("%s matches %d from node %d to %d ", __FILE__, matches.size(), ni->m_id, nj->m_id);
    if(pm->edge.informationMatrix(0, 0) == 10000)
    {
      ROS_WARN("Found new Tij with %d matches for edge from %d to %d", matches.size(), pre_id1, pre_id2); 
    }
    
    map<int, int>::iterator it = matches.begin(); 
    vector<cv::DMatch> mv(matches.size()); 
    int j= 0;
    while(it != matches.end())
    {
      cv::DMatch m; 
      m.trainIdx = it->first; 
      m.queryIdx = it->second; 
      mv[j++] = m; 
      ++it; 
    }
    
    pm->final_trafo = getTransformFromMatches(nj, ni, mv); 
    pm->edge.id1 = pre_id1; 
    pm->edge.id2 = pre_id2;
 
    return true; 

}

bool CGraphGT::bundleAdjust(MatchingResult* pm, CCameraNode* pNewNode, CamModel* pcam)
{
    int pre_id1 = pm->edge.id1; 
    int pre_id2 = pm->edge.id2; 
    correctMatchingID(pm); 
    
    CCameraNodeBA* ni = dynamic_cast<CCameraNodeBA*>(m_graph_map[pm->edge.id1]); 
    // CCameraNodeBA* nj = dynamic_cast<CCameraNodeBA*>(m_graph_map[pm->edge.id2]); 
    CCameraNodeBA* nj = dynamic_cast<CCameraNodeBA*>(pNewNode); 

    Eigen::Matrix4f Tji = pm->final_trafo.inverse(); 
    map<int, int> matches = nj->matchNodePairBA(ni, Tji, pcam); 

    if(matches.size() <= 4)
    {
      if(pm->edge.informationMatrix(0,0) == 10000)
      {
        ROS_ERROR("Nothing changed for edge from %d to %d", pre_id1, pre_id2); 
      }
      pm->edge.id1 = pre_id1; 
      pm->edge.id2 = pre_id2;
      return false; 
    }

    ROS_INFO("%s matches %d from node %d to %d ", __FILE__, matches.size(), ni->m_id, nj->m_id);
    if(pm->edge.informationMatrix(0, 0) == 10000)
    {
      ROS_WARN("Found new Tij with %d matches for edge from %d to %d", matches.size(), pre_id1, pre_id2); 
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

    g->push_back(PriorFactor<Pose3>(Symbol('s', 0), Pi, priorNoise)); 
    map<int, int>::iterator it = matches.begin(); 
    Values initialEstimate ; // = new Values; 

    // ROS_INFO("%d before insert into factor graph", __LINE__); 

    // insert pose 
    // noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
    initialEstimate.insert<Pose3>(Symbol('s',0), Pi);
    initialEstimate.insert<Pose3>(Symbol('s',1), Pj); 
    int j = 0;
    while(it != matches.end())
    {
      Eigen::Vector4f& pt = ni->m_feature_loc_3d[it->first]; 
      Point3 q ; q << pt(0), pt(1), pt(2); 

      initialEstimate.insert<Point3>(Symbol('u',j), q);
      Point2 measure_ni;
      Point2 measure_nj;
   
      measure_ni(0) = ni->m_feature_loc_2d[it->first].pt.x; 
      measure_ni(1) = ni->m_feature_loc_2d[it->first].pt.y; 

      measure_nj(0) = nj->m_feature_loc_2d[it->second].pt.x; 
      measure_nj(1) = nj->m_feature_loc_2d[it->second].pt.y; 
      
      // if(j == 0)
      {
        noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.014); 
        g->push_back(PriorFactor<Point3>(Symbol('u',j), q, pointNoise)); 
      }

      // ROS_INFO("%d before add measurement factor ", __LINE__);
      g->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_ni, measNoise, Symbol('s',0), Symbol('u', j), K)); 
      g->push_back(GenericProjectionFactor<Pose3, Point3, Cal3DS2>(measure_nj, measNoise, Symbol('s',1), Symbol('u', j), K)); 
      
      ++j; 
      ++it; 
      // ROS_INFO("%d add visual feature %d", __LINE__, j);
    }
    


    // isam.update(*g, initialEstimate); 
    // isam.update(); 
    // Values curEstimate = isam.calculateEstimate(); 

    LevenbergMarquardtOptimizer optimizer(*g, initialEstimate); 
    Values curEstimate = optimizer.optimize(); 

    Pj = curEstimate.at<Pose3>(Symbol('s',1)); 
    pm->final_trafo = Pj.matrix().cast<float>(); 
    Marginals marginals(*g, curEstimate, Marginals::CHOLESKY); 
    
    Matrix6 S_pose = marginals.marginalCovariance(Symbol('s', 1)); 
    pm->edge.informationMatrix = S_pose.inverse();  

    g->resize(0); 
    delete g; 
    initialEstimate.clear();
    
    pm->edge.id1 = pre_id1; 
    pm->edge.id2 = pre_id2;
    return true; 
}


bool CGraphGT::addToGTSAM(gtsam::NavState& new_state, int vid, bool add_pose )
{
  if(add_pose)
  {
    ROS_INFO("Line %d insert X(%d)", __LINE__, vid);
    mp_node_values->insert(X(vid), new_state.pose());
    mp_new_node->insert(X(vid), new_state.pose());
  }
  // ROS_INFO("Insert V(%d) and B(%d)", vid, vid);
  mp_node_values->insert(V(vid), new_state.v()); 
  mp_node_values->insert(B(vid), *mp_prev_bias); 
  mp_new_node->insert(V(vid), new_state.v()); 
  mp_new_node->insert(B(vid), *mp_prev_bias); 

  return true; 
}

bool CGraphGT::addToGTSAM(MatchingResult& mr, bool set_estimate)
{
  bool pre_exist = mp_node_values->exists(X(mr.edge.id1)); 
  bool cur_exist = mp_node_values->exists(X(mr.edge.id2));
  Pose3 inc_pose(mr.edge.transform.matrix()); 
  // inc_pose.print("inc_pose has value");
  // 

  // This is the bug, detected at Jan. 19, 2017, David Z
  // inc_pose = (*mp_u2c).inverse()*inc_pose*(*mp_u2c); 
  inc_pose = (*mp_u2c)*inc_pose*(*mp_u2c).inverse(); 

  if(!pre_exist && !cur_exist)
  {
    ROS_ERROR("%s two nodes %i and %i both not exist ", __FILE__, mr.edge.id1, mr.edge.id2); 
    return false; 
  }
  else if(!pre_exist)
  {
    ROS_WARN("this case is weired, has not solved it!");
    Pose3 cur_pose = mp_node_values->at<Pose3>(X(mr.edge.id2)); 
    Pose3 pre_pose = cur_pose * inc_pose.inverse(); 
    mp_node_values->insert(X(mr.edge.id1), pre_pose); 
    mp_new_node->insert(X(mr.edge.id1), pre_pose); 
  }
  else if(!cur_exist)
  {
    Pose3 pre_pose = mp_node_values->at<Pose3>(X(mr.edge.id1)); 
    Pose3 cur_pose = pre_pose * inc_pose;
    mp_node_values->insert(X(mr.edge.id2), cur_pose);
    mp_new_node->insert(X(mr.edge.id2), cur_pose); 
  }else if(set_estimate)
  {  
    // set estimate 
    // ROS_WARN("%s should not arrive here %i to set estimate", __FILE__, __LINE__);
    Pose3 pre_pose = mp_node_values->at<Pose3>(X(mr.edge.id1)); 
    Pose3 cur_pose = pre_pose * inc_pose; 
    mp_node_values->update(X(mr.edge.id2), cur_pose);
    mp_new_node->update(X(mr.edge.id2), cur_pose); 
  }

  // fake odo matrix 
  Eigen::Matrix<double, 6, 6> tmp = Eigen::Matrix<double, 6, 6>::Identity()*1000000; 
  // noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(tmp);

  Eigen::Matrix<double, 6, 6> Adj_Tuc = (*mp_u2c).AdjointMap(); 
  tmp = Adj_Tuc * mr.edge.informationMatrix * Adj_Tuc.transpose(); 

  // Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity(); 
  // Eigen::Matrix<double, 6, 6> cov = tmp.inverse(); 
  // cov = cov * tmp; 
  // if(!MatrixEqual(cov, I6, 1e-5))
  {
    // cout <<"what between id1: "<<mr.edge.id1<<" and "<<mr.edge.id2<<endl
    //  <<" Inf = "<<endl<<tmp<<endl
    //  <<" cov= "<<tmp.inverse()<<endl;
  }

  // noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(mr.edge.informationMatrix);
  noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(tmp);

  mp_fac_graph->add(BetweenFactor<Pose3>(X(mr.edge.id1), X(mr.edge.id2), inc_pose, visual_odometry_noise)); 
  mp_new_fac->add(BetweenFactor<Pose3>(X(mr.edge.id1), X(mr.edge.id2), inc_pose, visual_odometry_noise)); 
  
  return true; 
}

void CGraphGT::fakeOdoNode(CCameraNode* new_node)
{
   // assume id has been set 
   if( new_node->m_id != m_graph_map.size()) 
   { 
     cerr <<__FILE__<<" "<<__LINE__<<" Here this should not happen!" <<endl; 
     new_node->m_id = m_graph_map.size(); 
     new_node->m_seq_id = ++m_sequence_id; 
   }
   
   CCameraNode* pre_node = m_graph_map[new_node->m_id-1]; 
   MatchingResult mr; 
   mr.edge.id1 = pre_node->m_id; 
   mr.edge.id2 = new_node->m_id; 
   mr.edge.transform.setIdentity();  
   mr.edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Identity()*1e4; 
   addToGTSAM(mr, false); 
   
   m_graph_map[new_node->m_id] = new_node; 

   if(mb_record_vro_results)
   {
      recordVROResult(mr);  // record the result of VRO 
   }
   return ;
}


double CGraphGT::computeSdj(gtsam::Vector4& ni, gtsam::Matrix3& Sni, double Sdi, gtsam::Pose3* pose, gtsam::Matrix3 &St, 
                    gtsam::Vector4& nj)
{
    OrientedPlane3 Ni(ni);  
    Eigen::Vector3d nvi = Ni.normal().point3(); 

    // Pj = transform(Tij, Pi)
    Matrix36 D_Nj_D_Pij; 
    Matrix33 D_Nj_D_Ni; 
    OrientedPlane3 Nj = Ni.transform(*pose, D_Nj_D_Ni, D_Nj_D_Pij); 
    Point3 nvj = Nj.normal().point3();

    nj << nvj.x(), nvj.y(), nvj.z(), 0; 
    nj(3) = Nj.distance(); 

    Matrix3 I = Matrix3::Identity(); 
    Point3 tij = pose->translation(); 
    Vector3 vt; vt << tij.x(), tij.y(), tij.z(); 
    Vector3 D_dj_D_ni = (I - nvi*nvi.transpose())*vt; 

    // compute the variance of the distance between a point to the plane     
    double S_dj = Sdi + nvi.transpose()*St*nvi + D_dj_D_ni.transpose()*Sni*D_dj_D_ni; 
    return S_dj; 
}

bool CGraphGT::inThisPlane(CPlane* pj, double S_dj, double px, double py, double pz)
{
  // double px, py; 
  double d_thre = 0.014*0.014;
  // cam model 
  // the distance between this point to the estimated plane 
  double dis = pj->dis2plane(px, py, pz); 
  dis = dis * dis; 
  // test whether this point is an inlier 
  if(dis <= S_dj || dis <= d_thre) // inlier 
  {
    return true; 
  }
return false; 
}

bool CGraphGT::intensityTol(cv::Mat& img, int v1, int u1, int v2, int u2)
{
  int intensity_diff_threshold = 5; // 7  
  int i1 = img.at<unsigned char>(v1, u1); 
  int i2 = img.at<unsigned char>(v2, u2);
  
  if(i1 > i2 && i1 - i2 > intensity_diff_threshold) return false; 
  if(i1 <= i2 && i2 - i1 > intensity_diff_threshold) return false;
  return true; 
}

namespace{
  // region grow based on distance 
  void regionGrow(CGraphGT* pg, cv::Mat& flag, vector<int>& seed_u, vector<int>& seed_v, cv::Mat& img, cv::Mat& dpt, CPlane* pj, double S_dj, vector<int>& pj_index, CloudPtr& pts, vector<int>* new_added_pts)
  {
    typedef unsigned char uchar; 
    // current 
    int cu; int cv; double px, py, pz; 
    CamModel* pcam  = CamModel::gCamModel(); 
    cv::Mat visited = cv::Mat::zeros(flag.rows, flag.cols, CV_8UC1); 
    int i_thre = 20; 

    for(int i=0; i<seed_u.size(); i++)
    {
      cu = seed_u[i]; 
      cv = seed_v[i]; 
      if(visited.at<uchar>(cv, cu) == 2) continue; // already visited 

      visited.at<uchar>(cv, cu) = 2; 
      
      if(flag.at<uchar>(cv, cu) == INVALID_NUM) continue; 

      if(flag.at<uchar>(cv, cu) == 0) // check whether this point belongs to plane 
      {
        pz = dpt.at<unsigned short>(cv, cu)*pcam->m_z_scale; 
        pcam->convertUVZ2XYZ(cu, cv, pz, px, py, pz); 
        if(pg->inThisPlane(pj, S_dj, px, py, pz))
        {
          
          // if(cv <= 5)
          {
          // double dis = pj->dis2plane(px, py, pz); 
          // ROS_INFO("(u,v) = (%d %d), point: %f %f %f dis: %f S_dj: %f", cu, cv, px, py, pz, dis, S_dj);
          }

          flag.at<uchar>(cv, cu) = 1; // this point also in the plane id? 
          pj_index.push_back(cv*img.cols + cu); 
          Point pt; pt.x = px; pt.y = py; pt.z = pz; 
          pts->points.push_back(pt); 
          if(new_added_pts !=0 ) // for display 
          {
            new_added_pts->push_back(cv*img.cols + cu); 
          }
        }else
        {
            // this point is not in the plane, stop region grow
            flag.at<uchar>(cv, cu) == INVALID_NUM; 
            continue; 
        }
      }     
       
      // find the neighbors of cv, cu 
      int nv, nu; 
      // left 
      nv = cv; nu = cu - 1; 
      if(nu >= 0 && visited.at<uchar>(nv, nu) == 0 && pg->intensityTol(img, nv, nu, cv, cu))
      {
        seed_u.push_back(nu); seed_v.push_back(nv); 
        visited.at<uchar>(nv, nu) = 1;
      }
      // right 
      nv = cv; nu = cu + 1; 
      if(nu < flag.cols && visited.at<uchar>(nv, nu) == 0 && pg->intensityTol(img, nv, nu, cv, cu))
      {
        seed_u.push_back(nu); seed_v.push_back(nv);    
        visited.at<uchar>(nv, nu) = 1;
      }
      // top 
      nv = cv -1 ; nu = cu; 
      if(nv >= 0 && visited.at<uchar>(nv, nu) == 0 && pg->intensityTol(img, nv, nu, cv, cu))
      {  
        seed_u.push_back(nu); seed_v.push_back(nv); 
        visited.at<uchar>(nv, nu) = 1;
      }
      // down 
      nv = cv + 1; nu = cu; 
      if(nv < img.rows && visited.at<uchar>(nv, nu) == 0 && pg->intensityTol(img, nv, nu, cv, cu))
      {
        seed_u.push_back(nu); seed_v.push_back(nv); 
        visited.at<uchar>(nv, nu) = 1;
      }
    }
    return ; 
  }

  bool goodSeed(cv::Mat& flag, int uj, int vj)
  {
    if(uj < 0 || uj >= flag.cols || vj < 0 || vj >= flag.rows) // fall out of range 
    {
        return false; 
    }
      
    if(flag.at<uchar>(vj, uj) > 0) // already visited 
    {
      return false; 
    }
    return true; 
  }
}

// propogate plane information based on IMU prediction 
CPlaneNode* CGraphGT::predictPlaneNode(CPlaneNode* previous, cv::Mat& img, cv::Mat& dpt, gtsam::Pose3* pose, Matrix6& S_pose, vector<int>* new_added_pts)
{
  // transform from IMU frame to Camera frame 
  gtsam::Pose3 p; 
  p = (*mp_u2c).inverse()*(*pose)*(*mp_u2c); 
  Matrix6 Adj_Tcu = (*mp_u2c).inverse().AdjointMap(); 
  Matrix6 S_p = Adj_Tcu * S_pose * Adj_Tcu.transpose(); 
   
  // cam model 
  CamModel* pcam  = CamModel::gCamModel(); 

  // covariance for tij 
  Matrix3 S_tij = S_p.block<3,3>(3,3); 
  double d_thre = 0.014*0.014; // 0.014 = Sz for SR4000a

  // scale used for downsampling when compute plane's covariance 
  int downsample_scale = 4; // TODO: this should be parameterized 

  // 1, project the points in the previous plane node into current img 
  CPlaneNode* cur_P = new CPlaneNode();
  cv::Mat flag = cv::Mat::zeros(img.rows, img.cols, CV_8UC1); 

  // ROS_INFO("%s before propogate plane:", __FILE__); 
  int num_added = 0; // how many points have been added to a plane 
  for(int l=0; l<previous->mv_planes.size(); l++)
  {
    // Plane [ni, di, S_ni, S_di] 
    CPlane* plane = previous->mv_planes[l]; 
    Eigen::Vector4d nv; 
    nv << plane->nx_, plane->ny_, plane->nz_, plane->d1_; 
    // cout <<"plane "<<l<<" has nv: "<<endl<<nv<<endl;
    // double S_di = (*(plane->m_CP))(3,3); 
    // double S_di = plane->m_CP[3][3]; 
    double S_di = plane->m_E_Sdi; // estimated distance 
    Matrix3 S_ni; 
    plane->getNVCov(S_ni); 
    
    // predicted Plane [nj, dj, S_dj]
    Eigen::Vector4d nj; 
    double S_dj = computeSdj(nv, S_ni, S_di, pose, S_tij, nj); 
    CPlane* plane_j = new CPlane(); 
    plane_j->nx_ = nj(0); plane_j->ny_ = nj(1); plane_j->nz_ = nj(2); 
    plane_j->d1_ = nj(3); 

    // find the projected point who are in this plane 
    vector<int>& Pi_indices = previous->mv_indices[l]; 
    int index, ui, vi, uj, vj; 
    double px, py, pz; 
    float ujf, vjf; 
 
    // flag to show whether this pixel has been visited  
    vector<int> pj_u, pj_v; 
    vector<int> pj_index; 
    pj_u.reserve(Pi_indices.size()); 
    pj_v.reserve(Pi_indices.size()); 
    pj_index.reserve(Pi_indices.size()); 
    CloudPtr pj_pts(new Cloud); 
    pj_pts->points.reserve(Pi_indices.size()); 

    // ROS_INFO("%s before to find out how many points in this plane", __FILE__); 
    
    float du[3] = {-0.5, 0, 0.5}; 
    float dv[3] = {-0.5, 0, 0.5}; 

    for(int i=0; i<Pi_indices.size(); i++)
    {
      index = Pi_indices[i]; 

      // inverse project [u,v,d] -> [x, y, z]
      vi = index / img.cols;  ui = index - vi*img.cols; 
      if(ui <0 || ui > img.cols || vi <0 || vi > img.rows)
      {
        ROS_ERROR("%s what? ui = %d vi = %d index = %d", __FILE__, ui, vi, index);
      }else if(i <= 3){
        // ROS_INFO("process ui = %d vi = %d", ui, vi);
      }
      pz = previous->m_dpt.at<unsigned short>(vi, ui) * pcam->m_z_scale; 
      pcam->convertUVZ2XYZ(ui, vi, pz, px, py, pz); 

      // pj = Pji * pi 
      Point3 pi(px, py, pz); 
      Point3 pj = pose->transform_to(pi); 

      // project [x, y, z] -> [u, v]  
      pcam->convertXYZ2UV(pj.x(), pj.y(), pj.z(), ujf, vjf); 
      
      for(int m = 0; m<3; m++)
        for(int n =0; n<3; n++)
        {
          uj = (int)(ujf+du[m]); 
          vj = (int)(vjf+dv[n]); 
          if(!goodSeed(flag, uj, vj)) 
            continue;
          /*
             uj = (int)(ujf+0.5); vj = (int)(vjf+0.5); 

             if(!goodSeed(flag, uj, vj))
             {
             uj = (int)(ujf); vj = (int)(vjf); 
             if(!goodSeed(flag, uj, vj))
             {
             uj = (int)(ujf+0.5); vj = (int)(vjf); 
             if(!goodSeed(flag, uj, vj))
             {
             uj=(int)(ujf); vj = (int)(vjf+0.5); 
             if(!goodSeed(flag, uj, vj))
             continue; 
             }
             }
             }
             */

          // detect whether this point is on the plane 
          pz = dpt.at<unsigned short>(vj, uj) * pcam->m_z_scale; 

          if(pz <= 0.1 || pz >= 5) 
          {
            flag.at<uchar>(vj, uj) = INVALID_NUM; 
            continue; 
          }

          pcam->convertUVZ2XYZ(uj, vj, pz, px, py, pz);
          if(inThisPlane(plane_j, S_dj, px, py, pz))
          {
            // if(vj <= 5)
            {
              // double dis = plane_j->dis2plane(px, py, pz); 
              // ROS_INFO("(u,v) = (%d %d), point: %f %f %f dis: %f S_dj: %f", uj, vj, px, py, pz, dis, S_dj);
            }  

            pj_u.push_back(uj); 
            pj_v.push_back(vj); 
            flag.at<uchar>(vj, uj) = 1; 
            pj_index.push_back(vj*img.cols + uj); 
            Point pt; pt.x = pj.x(); pt.y = pj.y(); pt.z = pj.z(); 
            pj_pts->points.push_back(pt); 
          }else{
            flag.at<uchar>(vj, uj) = INVALID_NUM; 
          }
        }
    }
    // ROS_INFO("%s after find out points, before regionGrow seed size: %d", __FILE__, pj_index.size()); 
    // 1.2 update planes and add factor node to graph structure 
    
    // ROS_INFO("%s after region Grow, pj has points: %d", __FILE__, pj_index.size()); 

    // whether this plane is big enough 
    int thresh_pi = Pi_indices.size() * 0.7; 
    if(pj_index.size() > thresh_pi ) // TODO: parameterized 
    {
        // region grow to find all the points in this plane Pj
        regionGrow(this, flag, pj_u, pj_v, img, dpt, plane_j, S_dj, pj_index, pj_pts, new_added_pts); 

        // update plane parameters 
        plane_j->computeCOVSparse(img, pj_pts, pj_index, downsample_scale); 
        // add this plane to plane_node 
        cur_P->mv_planes.push_back(plane_j); 
        cur_P->mv_indices.push_back(pj_index); 
        cur_P->mv_landmark_id.push_back(previous->mv_landmark_id[l]); 
        ROS_WARN("%s detect a plane by plane propagation with landmark %d number of points %d, ", __FILE__, previous->mv_landmark_id[l], pj_index.size());
        if(cur_P->empty())
          cur_P->setDpt(dpt); 
    }
    num_added += pj_index.size(); 
  }
  
  // 2, use map structure to compute the rest points 
  int newly_added = 0; 
  int N = (img.rows*img.cols);
  int THRESH_N = N * 0.5; // 0.5 
  if(num_added > THRESH_N) // No need to find out another plane 
  {
    // TODO: 
    // ROS_INFO("%s added pts: %d, threshold_n %d", __FILE__, num_added, THRESH_N); 
  }else
  {
    // ROS_INFO("%s added pts: %d, try to find planes from the rest point cloud", __FILE__, num_added); 
    if(num_added  == 0) // previous node has no plane 
    {
      newly_added = cur_P->extractPlanes(img, dpt, pcam); 
      // ROS_INFO("%s added pts: = 0, find %d planes from the rest point cloud", __FILE__, newly_added); 
    }else{
      // 2.2 if many points are left, extract planes in the rest point cloud, find some new planes 
      CloudPtr rest_pts(new Cloud);  
      rest_pts->points.reserve(N - num_added); 
      double px, py, pz; 
      for(int v = 0; v<img.rows; v++)
        for(int u = 0; u<img.cols; u++)
        {
          if(flag.at<uchar>(v,u) == 0)
          {
            pz = dpt.at<unsigned short>(v, u) * pcam->m_z_scale; 
            pcam->convertUVZ2XYZ(u, v, pz, px, py, pz); 
            Point pt; pt.x = px; pt.y = py; pt.z = pz; 
            rest_pts->points.push_back(pt); 
          }
        }
      newly_added = cur_P->extractPlanes(rest_pts, pcam);
      ROS_INFO("%s find %d planes in the rest point cloud size = %d", __FILE__, newly_added, rest_pts->points.size()); 
    }
  }

  // 2.3 if new plane is extracted, add variable node and factor node into graph structure  
  if(newly_added)
  {
    // first merge overlapped planes in pj
    bool b_merged = cur_P->mergeOverlappedPlanes(newly_added); 
    // if(b_merged) 
       // ROS_INFO("yeah merge some planes!");

    if(cur_P->empty())
        cur_P->setDpt(dpt); 

    // TODO: doing plane association 
    
    // 
  }

  // 3, add the plane into landmark set, for future loop detection 
  // add to factor graph 

  return cur_P;
} 


void CGraphGT::firstPlaneNode(CPlaneNode* p)
{
  if(p!=NULL)
  {
    // add plane node 
    for(int i=0; i<p->mv_planes.size(); i++)
    {
      ROS_INFO("In firstPlaneNode add plane %d as landmark id %d", i, m_plane_landmark_id);
      p->mv_landmark_id[i] = m_plane_landmark_id; 
      addPlaneFactor(p->mv_planes[i], 0, m_plane_landmark_id++); 
    }
  }
  mv_plane_nodes[mv_plane_nodes.size()] = p; 
}

// make sure the pose node X(node_id) exist 
bool CGraphGT::addPlaneFactor(CPlane* p, int node_id, int landmark, double trace_SIMU)
{
  bool landmark_exist; 
  if(landmark < 0 )
  {
    ROS_ERROR("%s landmark should not be -1,",__FILE__); 
    landmark_exist = false; 
    return false;
  }
  else
    landmark_exist = mp_node_values->exists(L(landmark)); 
  
  bool pose_node_exist = mp_node_values->exists(X(node_id)); 
  if(!pose_node_exist)
  {
    ROS_ERROR("%s pose node %d not exist!", __FILE__, node_id);
    return false; 
  }

  // From Plane to OrientedPlane3 
  Eigen::Vector4d ni, nw; 
  ni << p->nx_, p->ny_, p->nz_, p->d1_; 
  OrientedPlane3 ONI(ni); 
  Eigen::Matrix<double, 3, 3> J; 
  Pose3 Tcu = mp_u2c->inverse(); 
  OrientedPlane3 ONJ = ONI.transform(Tcu, J); 
  const Eigen::Matrix<double, 3, 2>& ONI_Base = ONI.normal().basis(); 
  Eigen::Matrix<double, 2, 2> S_uni; 
  Eigen::Matrix<double, 3, 3> S_upi = Eigen::Matrix<double,3,3>::Identity(); 

again:
  // Plane parameters
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > S_pi(p->m_CP[0]); 
  Eigen::Matrix3d S_ni = S_pi.block<3,3>(0,0); 
  double S_di = S_pi(3,3); 

  S_uni = ONI_Base.transpose() * S_ni * ONI_Base; 
  
  // covariance 
  S_upi.block<2,2>(0,0) = S_uni; 
  S_upi(2,2) = S_di; 
  
  // OrientedPlane3 from camera frame to IMU frame 
  Eigen::Matrix<double, 3, 3> S_upj = J*S_upi*J.transpose(); 

  // cout <<"S_ni: "<<endl<<S_ni<<endl;
  // cout <<"S_upi: "<<endl<<S_upi<<endl;
  // cout <<"S_upj: "<<endl<<S_upj<<endl;

  if(!MatrixCheck(S_upj))
  {
    p->regularizeCOV(); 
    goto again; 
  }
  // if(S_di < 1e-6)
  //  S_upj(2,2) = S_upj(2,2) + 1e-5;
  Eigen::Matrix<double, 3, 3> S1 = S_upj.inverse(); 
  if(S1(0,0) != S1(0,0) || S1(0,0) < 0 || S1(1,1) < 0 || S_upj(0,0) < 0 || S_upj(1,1) < 0)
  {
    ROS_ERROR("what error here at node %d", node_id);
    cout<<"S1:"<<endl<<S1<<endl;

    if(!landmark_exist)
    {
    for(int i=0; i<=2; i++)
      S_upj(i,i) = 1e-6; 
    // if(S_upi(0,1) < 1e-9) 
      S_upj(0,1) = 0; 
    // if(S_upi(1,0) < 1e-9) 
      S_upj(1,0) = 0; 
    }else{
      return false; 
    }
  }

  double trace_SNV = p->getTraceSVN(); 

  if(!landmark_exist) // add variables 
  {
    // transform into 
    Pose3 Twu = mp_node_values->at<Pose3>(X(node_id)); 
    Pose3 Tuw = Twu.inverse(); 
    OrientedPlane3 ONW = ONJ.transform(Tuw); 
    mp_node_values->insert(L(landmark), ONW); 
    mp_new_node->insert(L(landmark), ONW); 
    mv_plane_num[landmark] = 0 ;
    if(trace_SNV > 1)
    {
      S_upj(0,0) = S_upj(1,1) = S_upj(2,2) = 1e-4;
      S_upj(1,0) = S_upj(0,1) = 0; 
    }
  }

  if(landmark_exist) // to see whether this landmark has been recently seen 
  {
    if(node_id - mv_plane_last_seen[landmark] <= 2) 
    {
      // increse the covariance   
      Pose3 Twu = mp_node_values->at<Pose3>(X(node_id)); 
      OrientedPlane3 ONW = mp_node_values->at<OrientedPlane3>(L(landmark)); 
      OrientedPlane3 pre_ONJ = ONW.transform(Twu); 
      Vector3 e = pre_ONJ.error(ONJ); 
      if(trace_SIMU == 0) 
        trace_SIMU = 1e-4; 
        // ROS_ERROR("%s what? trace_SIMU = %lf =0", __FILE__, trace_SIMU); 

      double k = trace_SNV; // /trace_SIMU; 
      if (k < 1) k = 1;
      else if(k >= 100) 
      {
        ROS_ERROR("%s discard plane observation here!", __FILE__); 
        return false; 
      }
      
      // double sigma_inc = (e(0)*e(0)*k*k + e(1)*e(1)*k*k)/2; 
      /*
      S_upj(0,0) += e(0)*e(0); //  sigma_inc; 
      S_upj(1,1) += e(1)*e(1); // sigma_inc; 
      S_upj(2,2) += 0 ; // sigma_inc; 
      S_upj(0,1) = S_upj(1,0) = 0; 
      ROS_WARN("%s Trace_SNV = %lf. Trace_IMU = %lf, k = %lf, e(0) = %lf e(1)=%lf e(2)=%lf", __FILE__, trace_SNV, trace_SIMU, k, e(0), e(1), e(2)); 
      */
      
    }
    if(trace_SNV > 1) 
      return false; 
  }
 
  // cout <<"S_upj: "<<endl<<S_upj<<endl;

  // for(int i=0; i<2; i++)
    // for(int j=0; j<2;j++)
        // S_upj(i,j) = (double)((unsigned int)(S_upj(i,j)*1e10))*1e-10;
  if(!DominateCheck(S_upj))
  {
    cout <<"S_upj is not triangle dominate, change it!"<<endl; 
    TriangleMatrix(S_upj); 
  }

  S_upj(0,1) = S_upj(1,0) = 0; 
  for(int i=0; i<3; i++)
    S_upj(i,i) = (float)((int)(S_upj(i,i)*1e8))*1e-8 + 1e-8;
  cout <<"S_upj: "<<endl<<S_upj<<endl;

  // double error_before = mp_fac_graph->error(*mp_node_values); 

  // add factor
  OrientedPlane3Factor plane_factor(ONJ.planeCoefficients(), noiseModel::Gaussian::Covariance(S_upj), X(node_id), L(landmark)); 

  double err_this_new_factor = plane_factor.error(*mp_node_values); 
  
  /*
  if(node_id - mv_plane_last_seen[landmark] <= 7) 
  {
      if(err_this_new_factor > 1e3)
      {
       ROS_ERROR("%s new plane factor increse err %lf, discard it", __FILE__, err_this_new_factor);
       return false; 
      }
  }else{
      if(err_this_new_factor > 10000)
      {
        plane_factor.print("plane with large error: ");
        Vector b = plane_factor.unwhitenedError(*mp_node_values); 
        ROS_ERROR("%s new plane factor increse err %lf unwhitederror: %lf %lf %lf, discard it", __FILE__, err_this_new_factor, b(0), b(1), b(2));
        return false;
      }
  }
  */

  mp_fac_graph->add(plane_factor); 
  mp_new_fac->add(plane_factor);

  // double error_after = mp_fac_graph->error(*mp_node_values); 

  mv_plane_last_seen[landmark] = node_id;  

  // ROS_ERROR("before add plane factor error : %lf after %lf increase %lf", error_before, error_after, error_after - error_before); 

  return true; 
}

set<int> CGraphGT::potentialPlaneNodes(int node_id)
{
  // look back for 10 nodes 
  int N_FOR_LOOK = 7; 
  set<int> r;  
  
  int from = m_graph_map.size()-1; 
  int to = from - N_FOR_LOOK; 
  if(to < 0 ) to = 0; 
  for(int id=from; id>=to; id--)
  {
    CPlaneNode* p = mv_plane_nodes[id]; 
    if(p == NULL) continue; 
    for(int i= 0 ; i < p->mv_landmark_id.size(); i++)
    {
      if(p->mv_landmark_id[i] < 0)
      {
        ROS_ERROR("At node %d get potential plane id %d, it has %d planes", id, p->mv_landmark_id[i], p->mv_planes.size());
      }
      r.insert(p->mv_landmark_id[i]);
    }
  }

  /*
  if(m_plane_landmark_id < N_FOR_LOOK)
  {
    for(int i=0; i<m_plane_landmark_id; i++)
      r.insert(i); 
    return r; 
  }


  for(int i=m_plane_landmark_id -1; i>=m_plane_landmark_id-N_FOR_LOOK ; i--)
    r.insert(i);
  
  // 
  std::map<int, int>::iterator it = mv_plane_num.begin(); 
  while(it != mv_plane_num.end())
  {
    if(it->second >= 10)
      r.insert(it->first); 
    ++it; 
  }*/
  return r; 
}

void CGraphGT::planeNodeAssociation(int node_id, CPlaneNode* p, double traceIMU)
{
  Pose3 pose = mp_node_values->at<Pose3>(X(node_id)); 

  static double COS15 = cos(15.*M_PI/180.);  
  static double COS10 = cos(10.*M_PI/180.);  
  static double COS30 = cos(30.*M_PI/180.); 
  static double COS5 = cos(5.*M_PI/180.);

  // find out 
  set<int> pls = potentialPlaneNodes(node_id); 
  Marginals marginals(*mp_fac_graph, *mp_node_values, Marginals::CHOLESKY); 
  
  // for debug
  ROS_WARN("check with previous planes: ");
  for(set<int>::iterator it = pls.begin(); it!=pls.end(); it++)
  {
    cout <<" "<<*it<<" ";
  }
  cout <<endl;

  for(int j=p->mv_planes.size()-1; j>=0; j--)
  {
    // ROS_INFO("mv_planes.size() %d j = %d", p->mv_planes.size(), j); 
    if(p->mv_landmark_id[j] >= 0) break;  
    CPlane* pi = p->mv_planes[j]; // plane in the body frame 
    int find_it = -1; 
    double mahalanobis_dis;
    set<int>::iterator it = pls.begin();
    Eigen::Vector4d ni; 
    ni << pi->nx_, pi->ny_, pi->nz_, pi->d1_;   
    cout <<"search for ni = "<<endl<<ni<<endl;
    while(it != pls.end())
    {
      int p_id = *(it); 
      if(p_id < 0 || p_id >= m_plane_landmark_id)
      {
        ROS_ERROR("node_id %d try to match with plane p_id = %d not exist",node_id, p_id);
      }
      OrientedPlane3 pw = mp_node_values->at<OrientedPlane3>(L(p_id));  
      
      Matrix36 D_Pj_D_pose; 
      Matrix33 D_Pj_D_pw;

      OrientedPlane3 Pj = pw.transform(pose, D_Pj_D_pw, D_Pj_D_pose);       
      
      // now Pj is in the IMU frame, transform it into camera frame 
      Pose3 Tcu = mp_u2c->inverse(); 
      Matrix33 D_Pc_D_pu; 
      Pj = Pj.transform(*(mp_u2c), D_Pc_D_pu); 
      Eigen::Vector4d nj = Pj.planeCoefficients(); 
      cout <<"check nj = "<<p_id<<endl<<nj<<endl;
      double COSA = ni.transpose()*nj - ni(3)*nj(3) ;
      if( COSA < 0)
      {
        cout<<"COA ="<<COSA<<" <0 ni*-1 = "<<endl<<ni<<endl;
        ni =-1 * ni; 
        COSA = -1*COSA; 
      }
      cout <<"COSA = "<<COSA<<" angle: "<<acos(COSA)*180./M_PI<<" dis = "<<fabs(ni(3)-nj(3))<<endl;

      // second strategy 

      if(COSA < COS10) {it++; continue; }
      if( fabs(ni(3)-nj(3)) <= 0.2)
      {
          find_it = p_id; 
          ROS_INFO("%s new plane matched with previous plane %d", __FILE__, find_it); 
          break;
      }
      
      // first strategy 
      /* if(COSA < COS30) {it++; continue;} 
      // if(fabs(ni(3)-nj(3)) > 1.) {it++; continue;}

      if(COSA >= COS10 && fabs(ni(3)-nj(3)) <= 0.2)
      {
        find_it = p_id; 
        // cout <<" new plane close enough to plane "<<find_it<<endl;
        ROS_INFO("%s new plane matched with previous plane %d", __FILE__, find_it); 
        break; 
      }
*/
      /*
      Matrix6 S_pose = marginals.marginalCovariance(X(node_id)); 
      Matrix3 S_pw = marginals.marginalCovariance(L(p_id)); 
      Matrix3 S_Pj = D_Pj_D_pw * S_pw * D_Pj_D_pw.transpose() + 
                  D_Pj_D_pose * S_pose * D_Pj_D_pose.transpose(); 

      S_Pj = D_Pc_D_pu * S_Pj * D_Pc_D_pu.transpose(); 

      // Plane parameters
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > S_pi(pi->m_CP[0]); 
      Eigen::Matrix3d S_ni = S_pi.block<3,3>(0,0); 
      double S_di = S_pi(3,3); 

      // From Plane to OrientedPlane3 
      OrientedPlane3 Pi(ni); 
      Eigen::Matrix<double, 2, 2> S_uni; 
      const Eigen::Matrix<double, 3, 2>& ONI_Base = Pi.normal().basis(); 
      S_uni = ONI_Base.transpose() * S_ni * ONI_Base; 

      // covariance 
      Eigen::Matrix<double, 3, 3> S_Pi = Eigen::Matrix<double,3,3>::Identity(); 
      S_Pi.block<2,2>(0,0) = S_uni; 
      S_Pi(2,2) = S_di; 

      // compute error Vector 
      Matrix3 D_eij_D_Pi, D_eij_D_Pj;
      Vector3 eij = Pi.errorVector(Pj, D_eij_D_Pi, D_eij_D_Pj); 
      
      Matrix3 S_eij = D_eij_D_Pi * S_Pi * D_eij_D_Pi.transpose() + 
        D_eij_D_Pj * S_Pj * D_eij_D_Pj.transpose(); 

      Matrix3 I_eij = S_eij.inverse(); 

      // cout <<"I_eij: "<<endl<<I_eij<<endl;

      mahalanobis_dis = eij(2)*eij(2)*I_eij(2,2); // eij.transpose() * I_eij * eij; 
      double Threshold_dis = 1.; // utils::chi2(3,1-0.9); 
      cout <<"mahalanobis_dis: "<<mahalanobis_dis<<" threshold_dis: "<<Threshold_dis<<endl;
      
      // if(mahalanobis_dis <= utils::chi2(3, 1-0.9))
      if(mahalanobis_dis <= Threshold_dis)
      {
        find_it = p_id; 
        ROS_WARN("%s M_dis good to match a previous plane %d", __FILE__, find_it); 
        break; 
      }else{
        ROS_INFO("failed to match ni with plane nj = %d", p_id); 
      }
*/
      it++; 
    }

    // succeed to match a plane variable
    if(find_it != -1)
    {
      // add the factor,  a new one 
      // TODO: debug here? 
      mv_plane_num[find_it]++; 
      ROS_WARN("%s match it with previous plane %d times %d ", __FILE__, find_it, mv_plane_num[find_it]); 
    }else{
      // a new one 
      find_it = m_plane_landmark_id++; 
      ROS_ERROR("%s failed to match this plane, add a new landmark %d dis is : %lf!", __FILE__, find_it, mahalanobis_dis); 
    }

    // ROS_INFO("%s before addPlaneFactor add plane factor id : %d", __FILE__, find_it); 
    p->mv_landmark_id[j] = find_it; 
    if(find_it < 0)
    {
      ROS_ERROR("%s at Line %d for node %d plane_id = %d",__FILE__, __LINE__, node_id, find_it);
    }
    addPlaneFactor(pi, node_id, find_it, traceIMU);
  }
  return; 
}

void CGraphGT::readVRORecord(std::string fname)
{
  return readVRORecord(fname, mv_vro_res); 
}

void CGraphGT::readVRORecord(std::string fname, std::vector<MatchingResult*> & mv)
{
  char buf[4096] = {0}; 
  ifstream inf(fname.c_str()); 
  int id_to, id_from; 
  double x,y,z,roll,pitch,yaw; 

  if(!inf.is_open())
  {
    cerr <<" failed to open file "<<fname<<endl;
    return ;
  }

  while(!inf.eof())
  {
    // Pose3 
    gtsam::Vector6 r; 
    MatchingResult* pm = new MatchingResult; 
    // inf>>id_to>>id_from>>x>>y>>z>>roll>>pitch>>yaw; 
    inf>>id_to>>id_from; 
    for(int i=0; i<6; i++)
      inf>>r(i); 
    gtsam::Pose3 p = gtsam::Pose3::ChartAtOrigin::Retract(r); 
    Eigen::Matrix4d md = p.matrix();   
    pm->final_trafo = md.cast<float>();
    pm->edge.transform.matrix() = md; 

    for(int i=0; i<6; i++)
      for(int j=i; j<6; j++)
      {
        inf>>pm->edge.informationMatrix(i,j); 
        pm->edge.informationMatrix(j,i) = pm->edge.informationMatrix(i,j);
      }
    pm->edge.id2 = id_to; 
    pm->edge.id1 = id_from; 
    
    // ROS_INFO("succeed to read record from %d to %d", id_from, id_to); 
    // if(pm->edge.informationMatrix(0,0) == 0) break; 
    if(inf.eof()) break; // For the last line in the file, after >> information(i,j), some whitespace remains which makes inf.eof() = false, and then add one wrong record
    mv.push_back(pm); 
    //if(mv.size() >= 5963)
    { 
      // cout <<" read vro record "<<id_to<<" "<<id_from<<" "<<r<<" information: "<<pm->edge.informationMatrix<<endl;
    }
  }

  cout <<__LINE__<<" read vro records "<<mv.size()<<endl;
  return ; 
}

void CGraphGT::printVROResult(ostream& ouf, MatchingResult& m)
{
  Eigen::Matrix<double, 6, 1> p = cov_Helper(m.final_trafo); 
  ouf<<m.edge.id2<<" "<<m.edge.id1<<" "<<p(0)<<" "<<p(1)<<" "<<p(2) <<" "<<p(3)<<" "<<p(4)<<" "<<p(5)<< " "; 
  for(int i=0; i<6; i++)
    for(int j=i;j<6;j++)
    {
      ouf<<m.edge.informationMatrix(i,j)<<" ";
    }
  ouf<<endl;
  ouf.flush();
  return;
}

void CGraphGT::recordVROResult(MatchingResult& m)  // record vro result 
{
  CCameraNode* pNow = m_graph_map[m.edge.id2];
  CCameraNode* pOld = m_graph_map[m.edge.id1]; 
  Eigen::Matrix<double, 6, 1> p = cov_Helper(m.final_trafo); 
  ofstream* pf = getRecFile(); 

  (*pf) << pNow->m_seq_id<<" "<<pOld->m_seq_id<<" "<<p(0)<<" "<<p(1)<<" "<<p(2) <<" "<<p(3)<<" "<<p(4)<<" "<<p(5)<< " "; 
  for(int i=0; i<6; i++)
    for(int j=i;j<6;j++)
    {
      (*pf)<<m.edge.informationMatrix(i,j)<<" ";
    }
  (*pf)<<endl;
  (*pf).flush();
  return ; 
}

// this function is to add new node in offline mode, which means the pose estimation has been known 
bool CGraphGT::addNodeOffline(CCameraNode* new_node, MatchingResult* mr, bool only_vo)
{
  bool ret = true;
  // it cannot be the first node  
  new_node->m_id = m_graph_map.size(); 
  new_node->m_seq_id = mr->edge.id2;
  CCameraNode* pre_node = m_graph_map[new_node->m_id-1]; 
  if(only_vo || mr->edge.informationMatrix(0,0) != 10000) // valid match
  {
    m_graph_map[new_node->m_id] = new_node; 
      
    // save previous 
    int pre_id1 = mr->edge.id1; 
    int pre_id2 = mr->edge.id2; 

    // ROS_INFO("Line %d before correction mr->id1 %d mr->id2 %d", __LINE__, mr->edge.id1, mr->edge.id2);
    correctMatchingID(mr); 
    // ROS_INFO("Line %d after correction mr->id1 %d mr->id2 %d", __LINE__, mr->edge.id1, mr->edge.id2);
    // printVROResult(std::cout, *mr);
    addToGTSAM(*mr, true); 

    // recover 
    mr->edge.id1 = pre_id1; 
    mr->edge.id2 = pre_id2;

  }else{ // not a valid match, skip this one 
    // 
    ret = false;
  }
  return ret;
}

// from sequence id to matching id 
void CGraphGT::correctMatchingID(MatchingResult* mr)
{
  int from_id = mr->edge.id1; 
  int to_id = mr->edge.id2; 
  map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
  bool from_good = false; 
  bool to_good = false; 
  while(it != m_graph_map.end())
  {
    if(it->second->m_seq_id == from_id)
    {
      mr->edge.id1 = it->second->m_id; 
      from_good = true;
    }
    if(it->second->m_seq_id == to_id) 
    {
      mr->edge.id2 = it->second->m_id; 
      to_good = true; 
    }
    if(from_good && to_good) break;
    ++ it; 
  }
  return ;
}

// add edge in offline mode, which means node has been added, and pose estimation has been known 
void CGraphGT::addEdgeOffline(MatchingResult* mr)
{
  if(mr->edge.informationMatrix(0,0) != 10000)
  {
    // save previous 
    int pre_id1 = mr->edge.id1; 
    int pre_id2 = mr->edge.id2; 

    correctMatchingID(mr); 
    addToGTSAM(*mr, false); 

    // recover 
    mr->edge.id1 = pre_id1; 
    mr->edge.id2 = pre_id2; 
  }
  return ;
}

ADD_RET CGraphGT::addNode(CCameraNode* new_node)
{
  // first node
  if(m_graph_map.size() == 0)
  {
    firstNode(new_node);
    return SUCC_KF; 
  }
  
  // 
  size_t old_node_size = mp_node_values->size(); 

  // match with previous node 
  new_node->m_id = m_graph_map.size(); 
  new_node->m_seq_id = ++m_sequence_id;
  CCameraNode* pre_node = m_graph_map[new_node->m_id-1]; 
  MatchingResult mr = new_node->matchNodePair(pre_node); 
  
  int current_best_match = 0; 
  if(mr.succeed_match) // found a trafo
  {
    if(isSmallTrafo(mr))// mr.edge.transform))  //  too small transform, e.g. < 3 degree or < 0.04 meters
    {
      // TODO: 
      // ROS_WARN("new_node %d is too close to its previous node, do not add into graph", new_node->m_seq_id);
      return FAIL_NOT_KF; 
    }else if(!isLargeTrafo(mr))// found a good match 
    {
       computeCovVRO(new_node, mr);  // compute the actual covariance 
       if(mr.edge.informationMatrix(0, 0) == mr.edge.informationMatrix(0,0))
       {
         addToGTSAM(mr, true);
         current_best_match = mr.inlier_matches.size(); 
         m_graph_map[new_node->m_id] = new_node; 
         if(mb_record_vro_results)
         {
           recordVROResult(mr);  // 
         }
       }
    }
  } else // failed_match 
  {
      ROS_ERROR("%s Found no transformation to predecessor", __FILE__);
  }

  // local loop closures 
  int seq_cand = CGTParams::Instance()->m_lookback_nodes; 
  QList<CCameraNode* > nodes_to_comp;   // nodes to compare 
  if(m_graph_map.size() > 3)
  {
    int n_id = new_node->m_id - 2;
    for(int j=0; j<seq_cand && n_id >= 0 ; j++ )
    {
      nodes_to_comp.push_back(m_graph_map[n_id--]); 
    }
  } 
  
  // Concurrently work on comparison 
  if(nodes_to_comp.size() > 0)
  {
    QThreadPool *qtp = QThreadPool::globalInstance(); 
    QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&CCameraNode::matchNodePair, new_node, _1)); 
    for(int i=0; i<results.size(); i++)
    {
      MatchingResult& mr = results[i]; 
      if(mr.succeed_match)
      {
        if(!isSmallTrafo(mr) && !isLargeTrafo(mr)) // not a too small and large transformation 
        {
          bool reset_estimate = mr.inlier_matches.size() > current_best_match; 
          if(reset_estimate) current_best_match = mr.inlier_matches.size(); 
          computeCovVRO(new_node, mr);     // compute the actual covariance 
          if(mr.edge.informationMatrix(0,0) != mr.edge.informationMatrix(0,0)) // nan information matrix 
            continue; 
          addToGTSAM(mr, reset_estimate);  // add factor and values 
          m_graph_map[new_node->m_id] = new_node;  // add into graph map
          if(mb_record_vro_results)
          {
            recordVROResult(mr); 
          }
        }
      }
    }
  }
  {//TODO: NonConcurrently 
  }
  
  // End of the function 
  bool b_found_trafo = this->mp_node_values->size() > old_node_size; 
  
  // TODO: keep the unmatched node? 
  if(b_found_trafo) 
    return SUCC_KF; 

  // return b_found_trafo;
  return FAIL_KF; 
}

void CGraphGT::optimizeGraphIncremental()
{
  mp_isam2->update(*mp_new_fac, *mp_new_node); 
  // mp_isam2->update(); 
  (*mp_node_values) = mp_isam2->calculateEstimate(); 
  // clear graph and nodes 
  mp_new_fac->resize(0); 
  mp_new_node->clear(); 
}


void CGraphGT::optimizeGraph()
{
  return CGraphGT::optimizeGraphBatch();
}

void CGraphGT::optimizeGraphBatch()
{
   LevenbergMarquardtOptimizer optimizer(*mp_fac_graph, *mp_node_values); 
   (*mp_node_values) = optimizer.optimize(); 
}

bool CGraphGT::isSmallTrafo(MatchingResult& mr)
{
  Eigen::Isometry3d& T = mr.edge.transform; 
  double dist = T.translation().norm(); 
  if(dist > CGTParams::Instance()->m_small_translation) return false; 

  double angle = acos((T.rotation().trace()-1)*0.5) * 180./M_PI; 
  if(angle > CGTParams::Instance()->m_small_rotation) return false; 

  return true;
}

bool CGraphGT::isLargeTrafo(MatchingResult& mr)
{
  Eigen::Isometry3d& T = mr.edge.transform; 
  double dist = T.translation().norm(); 
  if(dist > CGTParams::Instance()->m_large_translation) return true; 

  double angle = acos((T.rotation().trace()-1)*0.5) * 180./M_PI; 
  if(angle > CGTParams::Instance()->m_large_rotation) return true; 

  return false; 
}

size_t CGraphGT::camnodeSize()
{
  return m_graph_map.size(); 
}

bool CGraphGT::writeTrajectory(std::string f)
{
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    printf("%s failed to open f: %s to write trajectory!\n", __FILE__, f.c_str()); 
    return false; 
  }
  
  for(map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    Pose3 p = mp_node_values->at<Pose3>(X(it->first)); 
    p = (*mp_w2o)*p;
    // Vector3 rpy = p.rotation().rpy();
    // ouf<<it->first<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<rpy(0)<<" "<<rpy(1)<<" "<<rpy(2)<<" "<<it->second->m_seq_id<<endl;
    // Quaternion 
    gtsam::Quaternion q = p.rotation().toQuaternion(); 
    ouf<<it->first<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" "<<it->second->m_seq_id<<endl;
  }
  return true; 
}

bool CGraphGT::trajectoryPLY(std::string f, CG::COLOR c)
{
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    printf("%s %d failed to open f: %s to write trajectory!\n", __FILE__,__LINE__, f.c_str()); 
    return false; 
  }
  
  // first, add header 
  int vertex_number = m_graph_map.size(); 
  headerPLY(ouf, vertex_number);

  for(map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    Pose3 p = mp_node_values->at<Pose3>(X(it->first)); 
    p = (*mp_w2o)*p;
    ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<(int)CG::g_color[c][0]<<" "<<(int)CG::g_color[c][1]<<" "<<(int)CG::g_color[c][2]<<endl;
  }
  ouf.close(); 
  return true; 
}
/*
bool CGraphGT::mapPLY(std::string f, std::string img_dir, int skip, float depth_scale, CSparseFeatureVO* pSF)
{
  // generate a global point cloud 
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    printf("%s %d failed to open f: %s to write map!\n", __FILE__,__LINE__, f.c_str()); 
    return false; 
  }
  
  vector<float> pts_loc; 
  vector<unsigned char> pts_col;
  cv::Mat i_img, d_img; 
  CSReadCV r4k; 

  for(map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    Pose3 p = mp_node_values->at<Pose3>(X(it->first)); 
    p = (*mp_w2o)*p;
 
    // traverse all the points in this node 
    // get image 
    stringstream ss; 
    ss<<img_dir<<"/"<<setfill('0')<<setw(7)<<it->second->m_sequence_id<<".bdat";  // setw(4)
    r4k.readOneFrameCV(ss.str(), i_img, d_img);

    // compute local point cloud 
    vector<float> p_loc; 
    vector<unsigned char> p_col; 
    pSF->generatePointCloud(i_img, d_img, skip, depth_scale, p_loc, p_col); 

    // transform into global 
    Point3 fo, to; 
    for(int i=0; i<p_loc.size(); i+=3)
    {
      fo << p_loc[i], p_loc[i+1], p_loc[i+2]; 
      to = p.transform_from(fo); 
      p_loc[i] = to(0); p_loc[i+1] = to(1); p_loc[i+2] = to(2); 
    }
    
    pts_loc.insert(pts_loc.end(), p_loc.begin(), p_loc.end()); 
    pts_col.insert(pts_col.end(), p_col.begin(), p_col.end());

    // ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<g_color[c][0]<<" "<<g_color[c][1]<<" "<<g_color[c][2]<<endl;
  }
  
  // output into file 

  // first, add header 
  int vertex_number = pts_loc.size()/3; 
  headerPLY(ouf, vertex_number);

  for(int i=0; i<pts_loc.size(); i+=3)
  {
    ouf<< pts_loc[i]<<" "<<pts_loc[i+1]<<" "<<pts_loc[i+2]<<" "<<pts_col[i]<<" "<<pts_col[i+1]<<" "<<pts_col[i+2]<<endl;
  }
  ouf.close(); 
  return true;
}*/

void CGraphGT::headerPLY(std::ofstream& ouf, int vertex_number)
{
  ouf << "ply"<<endl
    <<"format ascii 1.0"<<endl
    <<"element vertex "<<vertex_number<<endl
    <<"property float x"<<endl
    <<"property float y"<<endl
    <<"property float z"<<endl
    <<"property uchar red"<<endl
    <<"property uchar green"<<endl
    <<"property uchar blue"<<endl
    <<"end_header"<<endl;
}

void CGraphGT::writeG2O(std::string f)
{
  gtsam::writeG2o(*mp_fac_graph, *mp_node_values, f); 
  return ;
}
