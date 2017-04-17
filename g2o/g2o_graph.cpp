

#include "g2o_graph.h"
#include <ros/ros.h>
#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include <fstream>
#include <utility>
#include <iostream>
#include <QList>
#include <QThread>
#include <QMutex>
#include <QtConcurrentMap>
#include <qtconcurrentrun.h>
#include "matching_result.h"
#include "camera_node.h"
#include "g2o_parameter.h"
#include "misc.h"

// using namespace g2o; 
using namespace std;

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > SlamBlockSolver; 
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver; 
// typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver; 
// typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*> VertexIDMap; 
 typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair; 
 typedef std::set<g2o::HyperGraph::Edge*> EdgeSet; 

CGraphG2O::CGraphG2O() : 
mp_optimizer(NULL)
{
  createOptimizer(); 
}
CGraphG2O::~CGraphG2O()
{
  for(map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    delete it->second; 
  }
  m_graph_map.clear();

  if(mp_optimizer)
  {
    mp_optimizer->clear(); 
    delete mp_optimizer; 
    mp_optimizer = 0; 
  }
}

void CGraphG2O::setWorld2Original(double p)
{
  // TODO: 
  m_w2o = tf::Transform(); 
}

g2o::SparseOptimizer* CGraphG2O::createOptimizer()
{
  if(mp_optimizer != NULL) delete mp_optimizer; 

  mp_optimizer = new g2o::SparseOptimizer(); 
  mp_optimizer->setVerbose(false); 
  
  SlamLinearCSparseSolver * linearSolver = new SlamLinearCSparseSolver(); 
  SlamBlockSolver* solver = new SlamBlockSolver(linearSolver); 
  g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver); 
  mp_optimizer->setAlgorithm(algo); 
  return mp_optimizer; 
}


void CGraphG2O::firstNode(CCameraNode* n)
{
  // 1, ids 
  n->m_id = m_graph_map.size(); 
  m_sequence_id = 0; 
  n->m_seq_id = ++ m_sequence_id;

  // 2, first Pose and add into graph 
  g2o::VertexSE3* reference_pose = new g2o::VertexSE3; 
  reference_pose->setId(n->m_id); 
  reference_pose->setFixed(true);
  mp_optimizer->addVertex(reference_pose); 
  m_graph_map[n->m_id] = n;
  return ;
}

bool CGraphG2O::addToGraph(MatchingResult& mr, bool set_estimate)
{
  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(mp_optimizer->vertex(mr.edge.id1)); 
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(mp_optimizer->vertex(mr.edge.id2)); 
  
  if(!v1 && !v2)
  {
    ROS_ERROR("%s two nodes %i and %i both not exist!", __FILE__, mr.edge.id1, mr.edge.id2); 
    return false;
  }else if(!v1)
  {
    ROS_WARN("this case is weired, has not solved it"); 
    v1 = new g2o::VertexSE3; 
    int v_id = mr.edge.id1; 
    v1->setId(v_id); 
    v1->setEstimate(v2->estimate() * mr.edge.transform.inverse()); 
    mp_optimizer->addVertex(v1);
  }else if(!v2)
  {
    v2 = new g2o::VertexSE3; 
    int v_id = mr.edge.id2; 
    v2->setId(v_id); 
    v2->setEstimate(v1->estimate() * mr.edge.transform); 
    mp_optimizer->addVertex(v2); 
  }else if(set_estimate){ 
    // set estimate 
    v2->setEstimate(v1->estimate() * mr.edge.transform);
  }

  g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3; 
  g2o_edge->vertices()[0] = v1; 
  g2o_edge->vertices()[1] = v2; 
  Eigen::Isometry3d meancopy(mr.edge.transform); 
  g2o_edge->setMeasurement(meancopy);
  // g2o_edge->setRobustKernal(&m_robust_kernel);
  g2o_edge->setInformation(mr.edge.informationMatrix); 
  mp_optimizer->addEdge(g2o_edge); 
  return true; 
}

void CGraphG2O::fakeOdoNode(CCameraNode* new_node)
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
   mr.edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Identity()*1e-3; 
   addToGraph(mr, false); 
   
   m_graph_map[new_node->m_id] = new_node; 
   return ;

}

ADD_RET CGraphG2O::addNode(CCameraNode* new_node)
{
  if(m_graph_map.size() == 0)
  {
    firstNode(new_node);
    return SUCC_KF; 
  }

    // 
  size_t old_node_size = camnodeSize(); 

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
    }else // found a good match 
    {
       addToGraph(mr, true); 
       m_graph_map[new_node->m_id] = new_node; 
       current_best_match = mr.inlier_matches.size(); 
    }
  } else // failed_match 
  {
      ROS_ERROR("%s Found no transformation to predecessor", __FILE__);
  }

  // local loop closures 
  int seq_cand = CG2OParams::Instance()->m_lookback_nodes; 
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
        if(!isSmallTrafo(mr)) // not a too small transformation 
        {
          bool reset_estimate = mr.inlier_matches.size() > current_best_match; 
          if(reset_estimate) current_best_match = mr.inlier_matches.size(); 
          addToGraph(mr, reset_estimate);  // add factor and values 
          m_graph_map[new_node->m_id] = new_node;  // add into graph map
        }
      }
    }
  }
  {//TODO: NonConcurrently 
  }
  
  // End of the function 
  bool b_found_trafo = this->camnodeSize() > old_node_size; 
  
  // TODO: keep the unmatched node? 
  if(b_found_trafo) 
    return SUCC_KF; 

  // return b_found_trafo;
  return FAIL_KF; 
}

void CGraphG2O::optimizeGraph()
{
  // TODO: parameterize
  int iter = 20; 
  int currIt = 0; 
  mp_optimizer->initializeOptimization(); 
  for(int i=0; i<iter; i+=currIt)
  {
    currIt = mp_optimizer->optimize(ceil(iter/10));
  }
  return ;
}

double CGraphG2O::error()
{
  mp_optimizer->computeActiveErrors();
  return mp_optimizer->chi2(); 
}


bool CGraphG2O::isSmallTrafo(MatchingResult& mr)
{
  Eigen::Isometry3d& T = mr.edge.transform; 
  double dist = T.translation().norm(); 
  if(dist > CG2OParams::Instance()->m_small_translation) return false; 

  double angle = acos((T.rotation().trace()-1)*0.5) * 180./M_PI; 
  if(angle > CG2OParams::Instance()->m_small_rotation) return false; 

  return true;
}


size_t CGraphG2O::camnodeSize()
{
  return m_graph_map.size(); 
}

void CGraphG2O::writeG2O(std::string f)
{
  ofstream ouf(f); 
  mp_optimizer->save(ouf); 
}

bool CGraphG2O::writeTrajectory(std::string f)
{
  ofstream ouf(f.c_str()); 
  if(!ouf.is_open())
  {
    ROS_ERROR("%s failed to open file : %s", __FILE__, f.c_str());
    return false; 
  }
  
  std::map<int, CCameraNode*>::iterator it = m_graph_map.begin(); 
  while(it != m_graph_map.end())
  {
    g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3*>(mp_optimizer->vertex(it->second->m_id) ); 
    
    tf::Transform pose = eigenTransf2TF(v->estimate());
    tf::Transform w2p = m_w2o * pose;
    
    ouf << it->second->m_id <<" "<< w2p.getOrigin().x() << " "<<w2p.getOrigin().y()<<" " <<w2p.getOrigin().z()
      <<" "<<w2p.getRotation().x()<<" "<<w2p.getRotation().y()<<" "<<w2p.getRotation().z()<<" "<<w2p.getRotation().w()<<" "<<it->second->m_seq_id<<endl;

    ++it;     
  }
  return true;
}

bool CGraphG2O::trajectoryPLY(std::string f, COLOR c)
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

    g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3*>(mp_optimizer->vertex(it->second->m_id) );    
    tf::Transform pose = eigenTransf2TF(v->estimate());
    tf::Transform w2p = m_w2o * pose;
    
    ouf << w2p.getOrigin().x() << " "<<w2p.getOrigin().y()<<" " <<w2p.getOrigin().z()<<" "<<(int)g_color[c][0]<<" "<<(int)g_color[c][1]<<" "<<(int)g_color[c][2]<<endl;
  }
  ouf.close(); 
  return true; 
}

void CGraphG2O::headerPLY(std::ofstream& ouf, int vertex_number)
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



