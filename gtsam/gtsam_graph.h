/*
 * Oct. 3, 2016, David Z 
 * 
 * Graph interface with gtsam 
 *
 * */

#ifndef GTSAM_GRAPH_H
#define GTSAM_GRAPH_H

#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <gtsam/base/Matrix.h>
#include "color.h"
// #include "opencv2/opencv.hpp"

namespace gtsam{
  class NonlinearFactorGraph;
  class Values;
  class Pose3; 
  class NavState;
  class ISAM2Params; 
  class ISAM2; 
  namespace imuBias
  {
  class ConstantBias;
  }
}

namespace cv{
class Mat; 
}
class CCameraNode; 
class CCameraNodeBA;
class CamModel; 
class MatchingResult; 
class CPlaneNode; // 
class CPlane;

typedef enum{SUCC_KF, FAIL_NOT_KF, FAIL_KF} ADD_RET; // return status for adding new node: SUCC_KF, succeed to add a kf; FAIL_NOT_KF, succeed, but not a kf, discard it; 
                                                     // FAIL_KF, fail to match.  

class CGraphGT 
{

public:
  CGraphGT(); 
  virtual ~CGraphGT();
  
  void firstNode(CCameraNode*, bool online=true);   // initialize everything 
  ADD_RET addNode(CCameraNode*);              // try to add a new node into graph 
  void fakeOdoNode(CCameraNode*);             // fake a Identity transformation node
  void optimizeGraph();                       // optimize the factor graph 
  void optimizeGraphBatch();
  void optimizeGraphIncremental(); 
  bool addToGTSAM(CCameraNodeBA* ni, CCameraNodeBA* nj, std::map<int, int>& matches, CamModel*); 
  bool vroAdjust(MatchingResult* , CCameraNode*, CamModel* );
  bool bundleAdjust(MatchingResult* , CCameraNode*, CamModel* ); 
  bool addToGTSAM(MatchingResult&, bool set_estimate);
  bool addToGTSAM(gtsam::NavState&, int vid, bool add_pose);  // add NavState(Pose, Velocity, Bias into GTSAM)
  void computeCovVRO(CCameraNode* pNew, MatchingResult& );       // compute the covariance of the VRO estimation
  void computeCovVRO(CCameraNode* pOld, CCameraNode* pNew, MatchingResult& );       // compute the covariance of the VRO estimation

  bool isSmallTrafo(MatchingResult&); 
  bool isLargeTrafo(MatchingResult&); 
  double error();                             // graph->error();
  size_t camnodeSize();                       // number of camera nodes 
  void writeGTSAM(std::string ouf);
  void writeG2O(std::string ouf);             // save trajectory into [g2o].txt Pose2 or Pose3   
  bool writeTrajectory(std::string ouf);      // dump trajectory into [ouf] 

  int m_sequence_id;                          // for each new node, m_sequence_id ++ 
  int m_vertex_id;                    // for each new node in the gtsam, m_vertex_id ++

  std::map<int, CCameraNode*> m_graph_map;    // associate node with its m_id 
  gtsam::NonlinearFactorGraph* mp_fac_graph;  // point to the factor graph
  gtsam::Values * mp_node_values;             // point to the node values in the factor graph
  void setWorld2Original(double r, double p, double y);   // 
  void setWorld2Original(double p); 
  void setCamera2IMU(double p); 
  void setCamera2IMUTranslation(double px, double py, double pz); // only set translation 
  gtsam::Pose3 * mp_w2o;      // transform from world to original (IMU)
  gtsam::Pose3 * mp_u2c;      // transform from IMU to camera 
  gtsam::imuBias::ConstantBias * mp_prev_bias; 
  gtsam::NavState * mp_prev_state;

  bool mb_record_vro_results;    // whether to record all the estimation from vro
  std::ofstream * getRecFile();       // record results from vro in this file 
  void recordVROResult(MatchingResult& m);  // record vro result 

  void printVROResult(std::ostream& ouf, MatchingResult& m); // printf matching result
  void readVRORecord(std::string inf);   // read vro record and save them into a vector 
  void readVRORecord(std::string inf, std::vector<MatchingResult*> & mv);   // read vro record and save them into a vector 
  
  // offline operation 
  std::vector<MatchingResult*> mv_vro_res;
  bool addNodeOffline(CCameraNode*, MatchingResult*, bool only_vo = false); 
  void addEdgeOffline(MatchingResult*); 
  // from sequence id to matching id 
  void correctMatchingID(MatchingResult* mr);
  
  // isam2 
  gtsam::ISAM2* mp_isam2; 
  gtsam::ISAM2Params * mp_isam2_param;
  gtsam::NonlinearFactorGraph* mp_new_fac;  // point to the new factor graph
  gtsam::Values * mp_new_node;    // point to the new node values in the factor graph
  void initISAM2Params(); 

  // ply 
  void headerPLY(std::ofstream&, int vertex_number); 
  bool trajectoryPLY(std::string ouf, CG::COLOR);   // output trajectory in ply 
  // bool mapPLY(std::string ouf, std::string img_dir, int skip, float depth_scale, CSparseFeatureVO* pSF);          // output 3d map in ply 


  // PLANES 
  // propogate plane information based on IMU prediction 
   CPlaneNode* predictPlaneNode(CPlaneNode* previous, cv::Mat& img, cv::Mat& dpt, gtsam::Pose3*, gtsam::Matrix6&  , std::vector<int>* added = 0); 
  
  // plane association 
   std::set<int> potentialPlaneNodes(int node_id); 
   void planeNodeAssociation(int node_id, CPlaneNode*, double trace_SIMU = 0); 
   std::map<int, CPlaneNode*> mv_plane_nodes;
   std::map<int, int> mv_plane_num;  // record the number that a plane has been observed 
   std::map<int, int> mv_plane_last_seen; // record which node recently see the plane 
   int m_plane_landmark_id;   // identical to the number of plane landmarks 
   int m_sift_landmark_id;    // identical to the number of sift landmarks

  // add plane variable and factor into graph 
   bool addPlaneFactor(CPlane* p, int pose_id, int landmark, double traceIMU = 0); 

  // process the first plane node 
   void firstPlaneNode(CPlaneNode* p); 

  // compute S_dk given [nj] = Rij'*ni, dj = ni'*tij + di; dk = nj'*pk + dj; 
  // input: ni, di, Sni, Sdi, pose = [wij, tij], 
  // output: nj, dj, J1 = D_ D_Nj_D_wij = hat(nj) return parts Sdj
   double computeSdj(gtsam::Vector4& ni, gtsam::Matrix3& Sni, double Sdi, gtsam::Pose3* p, gtsam::Matrix3 &St, gtsam::Vector4& nj); //

  // test wheather a point is on a plane 
  bool inThisPlane(CPlane* pj, double S_dj, double px, double py, double pz); 
  
   bool intensityTol(cv::Mat& img, int v1, int u1, int v2, int u2); 
// public:
 // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif
