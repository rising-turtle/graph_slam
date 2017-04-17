/*
 *  Feb. 10, 2017, David Z
 * 
 *  test plane feature difference given the pose change from vo
 *
 * */

#include <ros/ros.h> 
#include <sstream>
#include <string>
#include "SR_reader_cv.h"
#include "sparse_feature_vo.h"
#include "camera_node.h"
#include "cam_model.h"
#include "gtsam_graph.h"
#include "gt_parameter.h"
#include "imu_vn100.h"
#include "chi2.h"
#include "plane.h"
#include "plane_node.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>


using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 
using symbol_shorthand::L; // plane landmark (nv, d)

void init_parameters(); 
void test_with_sr4k(); 
void test_read_vro();

void copyMat(cv::Mat& f, cv::Mat& t);

string g_file_dir; 
string g_file_pre; 
string g_file_suf;
string g_data_name;
string g_out_log;
string g_match_file; 

bool g_chi2_test = false;   // whether to use Chi2 test to delete VRO edges 
bool g_plane_factor = false; // whether to add plane factors into graph 
bool g_view_plane = false;  // whether to view the result of plane propagation 
int g_f_start; 
int g_f_end;

string g_imu_file; // imu measurement file 
string g_imu_time_file; // sr timestamp file, which is used to synchronize between camera data and imu data 
string g_vro_results_file; // where the vro results are stored 

bool loadImgTime(map<int, double>&); // load synchronized time stamps

double computePlaneDis(CPlane* pi, CPlane* pj, Pose3& Tij, Matrix6& Sij, double &);  // compute the M distance between (pj, Tij.transform(pi))

double computePlaneNodeDis(CPlaneNode* Pi, CPlaneNode* Pj, Pose3& Tij, Matrix6& Sij, double&); 

int main(int argc, char* argv[])
{
  // CImuBase::getParam(); 
  ros::init(argc, argv, "test_vro_imu_graph"); 
  ros::NodeHandle n; 
  
  ROS_INFO("at line %s before test_with_sr4k", __FILE__);

  test_with_sr4k(); 
  // test_read_vro(); 

  return 0; 
}

void test_with_sr4k()
{
  // parameters 
  init_parameters(); 
  // std::string fname = "/home/david/.ros/vro_results_backup.log";   
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  CCameraNode::set_cam_cov(sr4k); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;
  sr4k.setDepthScale(depth_scale); 
  CamModel::updategCamModel(sr4k); 

  CSparseFeatureVO spr_vo(sr4k);
  CSReadCV r4k; 

  // set graph strcuture 
  CGraphGT gt_graph; 
   // VRO result 
  gt_graph.readVRORecord(g_vro_results_file); 
  // gt_graph.setWorld2Original(D2R(CGTParams::Instance()->m_initial_pitch));
  gt_graph.setCamera2IMU(0); 

  // extract intensity and depth img for each sr_data 
  cv::Mat i_img, d_img; // 

  ROS_INFO("start the first node process");
  // add first node 
  stringstream ss; 
  ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<g_f_start<<"."<<g_file_suf;  // setw(4)
  r4k.readOneFrameCV(ss.str(), i_img, d_img);

  CCameraNode* pNewNode = new CCameraNode();
  spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
  pNewNode->m_seq_id = g_f_start;
  gt_graph.firstNode(pNewNode, false);

  // traversely visit the vro matching results 
  int cur_frame_id = g_f_start; 
  int cur_imu_id = cur_frame_id; 
  int cur_node_id = cur_imu_id; 
  
  ROS_INFO("at line %d start to read vro results", __LINE__);

  // record consequent NavState 
  NavState pre_state, cur_state; 
  cv::Mat pre_rgb, pre_dpt; 

  pre_rgb = i_img.clone(); 
  pre_dpt = d_img.clone(); 

  // if this node is removed due to Chi2 check, then the related local loop edges are removed also
  bool node_removed_by_chi2 = false; 

  ofstream ouf(g_out_log.c_str()); 
  ifstream inf(g_match_file.c_str()); 
  int id1, id2, num; 
  double err, err_raw; // plane difference error 

  for(int i=0; i<gt_graph.mv_vro_res.size() && i < g_f_end && ros::ok(); i++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[i]; 
    inf >> id2 >> id1 >> num;  // load match result 
    ROS_INFO("handle id1 = %d id2 = %d pm->edge.id1 = %d pm->edge.id2 = %d", id1, id2, pm->edge.id1, pm->edge.id2);
    ROS_INFO("handle i = %d g_f_end = %d vro_size(): %d", i, g_f_end, gt_graph.mv_vro_res.size());

    // if(id2 != 1429 || id1 != 1427) continue; 
    
    if(pm->edge.id2 <= g_f_start) continue; // skip edges

    if(pm->edge.id2 > cur_frame_id) // a new frame is aligned, an incremental edge is added 
    {
      cur_imu_id = pm->edge.id2;
      // ROS_INFO("process sequence node id : %d", cur_imu_id);
      stringstream ss; 
      ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<pm->edge.id2<<"."<<g_file_suf;  // setw(4)
      err = err_raw = 0; 

      if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
      {
        CCameraNode* pNewNode = new CCameraNode();
        spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
        gt_graph.addNodeOffline(pNewNode, pm, true);

        Matrix4 inc_T = pm->final_trafo.cast<double>(); 
        Pose3 inc_p (inc_T); 
        Matrix6 cov = pm->edge.informationMatrix.inverse(); 

        CPlaneNode * prePNode = NULL; 
        CPlaneNode * curPNode = NULL; 
        if(pm->edge.informationMatrix(0,0) == 10000) // failed to add vo factor, so the pose node has not been added,        
        {
          // manually add node into graph 
          gt_graph.m_graph_map[pNewNode->m_id] = pNewNode; 
          
          ouf << id2<<"\t"<<id1<<"\t"<<num <<"\t"<< err<<"\t"<<err_raw <<endl; 
        }else{
          if(num <= 20)
          {
            prePNode = new CPlaneNode(); 
            int ret = prePNode->extractPlanes(pre_rgb, pre_dpt, &sr4k); 
            if(ret == 0) // hopeless case
            {
              ROS_INFO("%d no plane detected, delete it", __LINE__); 
              delete prePNode; prePNode = NULL;
            }else{
              curPNode = new CPlaneNode();
              ret = curPNode->extractPlanes(i_img, d_img, &sr4k); 
              if(ret == 0)
              {
                ROS_INFO("%d cur planeNode is NULL, delete it", __LINE__);
                delete curPNode; curPNode = NULL; 
              }else
              {
                ROS_INFO("before computePlaneNodeDis"); 
                err = computePlaneNodeDis(prePNode, curPNode, inc_p, cov, err_raw); 
                ROS_INFO("after computePlaneNodeDis");
              }
            }
          }else{
            err = err_raw = 0;
          }
          /*
          // get pose change,
          int pre_node_id = pNewNode->m_id - 1; 
          // previous plane Node 
          CPlaneNode * prePNode = gt_graph.mv_plane_nodes[pre_node_id];

          if(prePNode == NULL) // try to detect planes at previous node 
          {
            prePNode = new CPlaneNode(); 
            ROS_INFO("%d get preNode = NULL, try to extractPlanes", __LINE__); 
            int ret = prePNode->extractPlanes(pre_rgb, pre_dpt, &sr4k); 
            if(ret == 0) // hopeless case
            {
              ROS_INFO("%d no plane detected, delete it", __LINE__); 
              delete prePNode; prePNode = NULL;
            } else{ 
              gt_graph.planeNodeAssociation(pNewNode->m_id-1, prePNode); 
            }        
          }
          
          Matrix4 inc_T = pm->final_trafo.cast<double>(); 
          Pose3 inc_p (inc_T); 
          Matrix6 cov = pm->edge.informationMatrix.inverse(); 

          if(prePNode == NULL)
          {
            // no plane is detected 
            
          }else{
                            
            ROS_INFO("%d pre planeNode is not NULL, try to predictPlaneNode", __LINE__);   
            curPNode = gt_graph.predictPlaneNode(prePNode, i_img, d_img, &inc_p, cov);  
            // add plane factors 
            bool b_has_new_plane = false; 
            double plane_dis_err; 
            double plane_raw_dis_err;
            if(curPNode != NULL)
            {
              for(int j=0; j<curPNode->mv_planes.size(); j++)
              {
                ROS_INFO("%d cur planeNode has planes, add plane %i", __LINE__, j);
                int idj = curPNode->mv_landmark_id[j]; 
                // ROS_INFO("curPlane has plane %d landmark_id %d", i, curPlane->mv_landmark_id[i]);
                if(idj < 0) 
                {
                  b_has_new_plane = true; 
                  break; 
                }
                int k =0; 
                int idk = -1; 
                for(; k<prePNode->mv_planes.size(); k++)
                {
                  idk = prePNode->mv_landmark_id[k];  
                  if( idk == idj)
                    break; 
                }
                
                if(idk != idj)
                {
                  ROS_ERROR("what? idi (%d) != idj (%d)", idk, idj); 
                }

                // compute plane dis
                CPlane* pi = prePNode->mv_planes[k]; 
                CPlane* pj = curPNode->mv_planes[j]; 
                
                plane_dis_err = computePlaneDis(pi, pj, inc_p, cov, plane_raw_dis_err);
                if(plane_dis_err > err)
                  err = plane_dis_err; 
                if(plane_raw_dis_err > err_raw)
                  err_raw = plane_raw_dis_err;

              }

              if(b_has_new_plane)
              {
                  gt_graph.planeNodeAssociation(pNewNode->m_id, curPNode); 
              }

            }else{
              ROS_INFO("arrive here, no idea why I am here.");  
            }
            
            if(curPNode!= NULL && curPNode->mv_planes.size() == 0)
            {
              delete curPNode; curPNode = NULL;
            }
          } // prePNode = NULL
          */
          ouf << id2<<"\t"<<id1<<"\t"<<num <<"\t"<< err<<"\t"<<err_raw <<endl; 
        }// valid match 

          gt_graph.mv_plane_nodes[pNewNode->m_id] = curPNode; // no plane is needed
          pre_rgb = i_img.clone(); 
          pre_dpt = d_img.clone(); 
    
          cur_frame_id = cur_imu_id; 
      }else{
        ROS_ERROR("%s failed to load frame %d", __FILE__, pm->edge.id2);
        break; 
      }

    }else // a loop closure edge is found 
    {
      // add all loops into the graph 
      int j = i+1; 
      // ROS_INFO("First loop edge pm->edge.id2: %d id1: %d i= %d cur_frame_id = %d", pm->edge.id2, pm->edge.id1, i, cur_frame_id);
      while(j < gt_graph.mv_vro_res.size())
      {
        pm = gt_graph.mv_vro_res[j]; 
        if(pm->edge.id2 > cur_frame_id)
        {
          break; 
        }
        inf >> id2 >> id1 >> num;  // load match result 
        j++; 
      }

      i = j - 1; // update i
    }
  }

  return ;
}

double computePlaneNodeDis(CPlaneNode* pni, CPlaneNode* pnj, Pose3& Tij, Matrix6& S_Tij, double& err_raw)
{
    double COS10 = cos(10.*M_PI/180.); 
    double err = 0; 
    err_raw = 0; 
    int best_i = -1, best_j = -1; 
    int i, j;
    Vector4 bnej; 

    // find matches 
    for(i=0; i<pni->mv_planes.size(); i++)
    {
      CPlane* pi = pni->mv_planes[i]; 
      CPlane* pj = NULL; 
      Vector4 ni; ni << pi->nx_, pi->ny_, pi->nz_, pi->d1_; 
      OrientedPlane3 Pi(ni); 
      OrientedPlane3 PEj = Pi.transform(Tij); 
      Vector4 nej = PEj.planeCoefficients(); 
      cout<<"i = "<<i<<" pej: "<<endl<<nej<<endl;
      bool found_it = false; 
      for(j=0; j<pnj->mv_planes.size(); j++)
      {
        pj = pnj->mv_planes[j]; 
        Vector4 nj; nj << pj->nx_, pj->ny_, pj->nz_, pj->d1_; 
        cout <<"j = "<<j <<" nj: "<<endl<<nj<<endl; 
        double COSA = nej.transpose()*nj - nej(3)*nj(3); 
        cout <<"COSA = "<<COSA<<" angle = "<<acos(COSA)*180./M_PI<<endl;
        if(fabs(COSA) >= COS10 && fabs(nej(3)- nj(3)) <= 0.2)
        {
          // got it  
          found_it = true; 
          bnej = nej; 
          break; 
        }
      }
      if(found_it)
      {
        double p_dis, p_raw_dis; 
        p_dis = computePlaneDis(pi, pj, Tij, S_Tij, p_raw_dis); 
        if(p_dis > err)
        {
          err = p_dis; 
          err_raw = p_raw_dis; 
          best_i = i; 
          best_j = j; 
          ROS_INFO("err = %lf, err_raw = %lf, at i = %d j = %d", err, err_raw, i, j);
        }
      }
    }

  return err; 
}



double computePlaneDis(CPlane* pi, CPlane* pj, Pose3& Tij, Matrix6& Sij, double& raw_e)  // compute the M distance between (pj, Tij.transform(pi))
{
  // Handle Pi 

  // Plane parameters
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > S_pi(pi->m_CP[0]); 
  Eigen::Matrix3d S_ni = S_pi.block<3,3>(0,0); 
  double S_di = S_pi(3,3); 
  Vector4 ni; ni << pi->nx_, pi->ny_, pi->nz_, pi->d1_; 

  // From Plane to OrientedPlane3 
  OrientedPlane3 Pi(ni); 
  Eigen::Matrix<double, 2, 2> S_uni; 
  const Eigen::Matrix<double, 3, 2>& ONI_Base = Pi.normal().basis(); 
  S_uni = ONI_Base.transpose() * S_ni * ONI_Base; 

  Matrix36 D_Pj_D_Tij; 
  Matrix33 D_Pj_D_pi;
  OrientedPlane3 Pj_E = Pi.transform(Tij, D_Pj_D_pi, D_Pj_D_Tij ); 
  
  // covariance 
  Eigen::Matrix<double, 3, 3> S_Pi = Eigen::Matrix<double,3,3>::Identity(); 
  S_Pi.block<2,2>(0,0) = S_uni; 
  S_Pi(2,2) = S_di; 

  Eigen::Matrix<double, 3, 3> S_PjE = Eigen::Matrix<double,3,3>::Identity(); 
  S_PjE = D_Pj_D_Tij * Sij * D_Pj_D_Tij.transpose() + D_Pj_D_pi * S_Pi * D_Pj_D_pi.transpose();
  // S_PjE.block<2,2>(0,0) = S_uni; 
  // S_PjE(2,2) = S_di; 

  // Handle Pj 
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > S_pj(pj->m_CP[0]); 
  Eigen::Matrix3d S_nj = S_pj.block<3,3>(0,0); 
  double S_dj = S_pj(3,3); 
  Vector4 nj; nj << pj->nx_, pj->ny_, pj->nz_, pj->d1_; 

  // From Plane to OrientedPlane3 
  OrientedPlane3 Pj(nj); 
  Eigen::Matrix<double, 2, 2> S_unj; 
  const Eigen::Matrix<double, 3, 2>& ONI_Basej = Pj.normal().basis(); 
  S_unj = ONI_Basej.transpose() * S_nj * ONI_Basej; 
  
  // covariance 
  Eigen::Matrix<double, 3, 3> S_Pj = Eigen::Matrix<double,3,3>::Identity(); 
  S_Pj.block<2,2>(0,0) = S_unj; 
  S_Pj(2,2) = S_dj; 


  // compute error Vector 
  Matrix3 D_eij_D_PjE, D_eij_D_Pj;
  Vector3 eij = Pj_E.errorVector(Pj, D_eij_D_PjE, D_eij_D_Pj); 
  raw_e = eij.transpose()*eij; 

  Matrix3 S_eij = D_eij_D_PjE * S_PjE * D_eij_D_PjE.transpose() + 
    D_eij_D_Pj * S_Pj * D_eij_D_Pj.transpose(); 

  Matrix3 I_eij = S_eij.inverse(); 

  double mahalanobis_dis = eij.transpose() * I_eij * eij; 

  return mahalanobis_dis; 

}

void init_parameters()
{
  ros::NodeHandle np("~"); 
  // np.setParam("sr_data_file_dir", "/media/david/work/work/data/SLAM/SR4000/dataset_82"); 
  // np.setParam("sr_start_frame", 1); 
  // np.setParam("sr_end_frame", 2270);
  // np.setParam("sr_new_file_version", false);

  // nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  // np.param("sr_data_file_dir", g_file_dir, string("/home/davidz/work/data/SwissRanger4000/exp/dataset_82")); // default parameters 
  np.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5")); // default parameters 
  np.param("sr_data_prefix", g_file_pre, string("d1")); 
  np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_f_end, 200); 
  np.param("sr_data_name", g_data_name, string("etas_f5"));

  np.param("imu_file", g_imu_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/imu_v100.log"));
  np.param("imu_time_file", g_imu_time_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k/timestamp.log")); 
  np.param("vro_results_file", g_vro_results_file, string("vro_results.log_hybrid")); 
  np.param("chi2_for_vro", g_chi2_test, g_chi2_test); 
  np.param("plane_aided", g_plane_factor, true);
  np.param("view_plane", g_view_plane, g_view_plane); 

  CGTParams* p = CGTParams::Instance(); 
  p->m_small_translation = 0.04; 
  p->m_small_rotation = 3; 
  p->m_lookback_nodes = 5; 
  p->m_optimize_step = 10; 
  np.param("vo_small_translation", p->m_small_translation, 0.04); 
  np.param("vo_small_rotation"  ,  p->m_small_rotation, 3.); 
  np.param("gt_optimize_step",     p->m_optimize_step, 10); 
  np.param("gt_lookback_nodes",    p->m_lookback_nodes, 5);
  np.param("gt_output_dir",        p->m_output_dir, std::string("./"));
  np.param("gt_initial_pitch",     p->m_initial_pitch, -14.7);

  np.param("result_log",  g_out_log, string("num_vs_err.log")); 
  np.param("matched_num_log", g_match_file, string("input_match.log"));
}





