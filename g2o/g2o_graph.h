/*
 * Oct. 17, 2016, David Z 
 * 
 * Graph interface with g2o
 *
 * */

#ifndef G2O_GRAPH_H
#define G2O_GRAPH_H

#include <map>
#include <tf/tf.h>
#include "color.h"

namespace g2o{ 
  class SparseOptimizer; 
}
class CCameraNode; 
class MatchingResult; 

typedef enum{SUCC_KF, FAIL_NOT_KF, FAIL_KF} ADD_RET; // return status for adding new node: SUCC_KF, succeed to add a kf; FAIL_NOT_KF, succeed, but not a kf, discard it;                                                                   // FAIL_KF, fail to match.  
class CGraphG2O 
{

public:
  CGraphG2O(); 
  virtual ~CGraphG2O();
  
  g2o::SparseOptimizer* createOptimizer(); 

  void firstNode(CCameraNode*);               // initialize everything 
  ADD_RET addNode(CCameraNode*);              // try to add a new node into graph 
  void fakeOdoNode(CCameraNode*);             // fake a Identity transformation node
  void optimizeGraph();                       // optimize the factor graph 
  bool addToGraph(MatchingResult&, bool set_estimate);
  bool isSmallTrafo(MatchingResult&); 
  double error();                             // graph->error();
  size_t camnodeSize();                       // number of camera nodes 
  void writeG2O(std::string ouf);             // save trajectory into [g2o].txt Pose2 or Pose3   
  bool writeTrajectory(std::string ouf);      // dump trajectory into [ouf] 

  int m_sequence_id;                          // for each new node, m_sequence_id ++ 

  std::map<int, CCameraNode*> m_graph_map;    // associate node with its m_id 
  g2o::SparseOptimizer * mp_optimizer;  // point to the factor graph
  // void setWorld2Original(double r, double p, double y);   // 
  void setWorld2Original(double p); 
  tf::Transform m_w2o; 

  void headerPLY(std::ofstream&, int vertex_number); 
  bool trajectoryPLY(std::string ouf, COLOR);   // output trajectory in ply 
};

#endif
