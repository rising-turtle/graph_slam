/*
 * test gtsam graph slam in offline mode with imu preintegration 
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
#include "imu_MEMS.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 

void init_parameters(); 
void test_with_sr4k(); 
vector<int> load_syn_camera_time();

string g_file_dir; 
string g_file_pre; 
string g_file_suf;
int g_f_start; 
int g_f_end;

string g_syn_camera_imu;    // file contain camera time data 
string g_imu_data_dir;      // where to read the imu data
int g_imu_id_first_camera;  // imu id corresponds to the first camera data 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_gt_imu_MEMS"); 
  ros::NodeHandle n; 

  test_with_sr4k(); 

  return 0; 
}

void test_with_sr4k()
{
  // read data 
  init_parameters(); 
  CSReadCV r4k; 
  // if(!r4k.loadAllData()) // sometimes failed due to memory size limitation 
  {
  //  ROS_WARN("%s no failed to load sr4k data!", __FILE__);
  //  return ; 
  }

  // set camera model and sparse feature module 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;
  CSparseFeatureVO spr_vo(sr4k);
  
  // set graph strcuture 
  CGraphGT gt_graph; 
  gt_graph.setWorld2Original(D2R(CGTParams::Instance()->m_initial_pitch));
  
  // extract intensity and depth img for each sr_data 
  cv::Mat i_img, d_img; // 

  // imu setup 
  CImuMEMS* imu_measure = new CImuMEMS; 
  if(!imu_measure->readImuData(g_imu_data_dir))
  {
    ROS_ERROR("Failed to read imu data from %s", g_imu_data_dir.c_str()); 
    return ; 
  }

  imu_measure->computePriorBias(); // compute bias first 
  vector<int> camera_time_set = load_syn_camera_time(); 

  int imu_next_id = g_imu_id_first_camera; // imu_camera_syn_id 
  ROS_WARN("imu_id with the first camera is %d", imu_next_id);
  double time_elapsed = 0; 
  NavState curr_state; 
  imuBias::ConstantBias curr_bias; 

  bool cur_pose_has_been_added = true; 
  int cur_node_id = -1; 

  for(int i=g_f_start; i<g_f_end; ++i)
  {
    // get current img 
    stringstream ss; 
    ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(4)<<i<<"."<<g_file_suf; 

    // while(r4k.getCurrentFrameCV(i_img, d_img))  // get current img 
    if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
    {
      // imu time synchronization 
      time_elapsed = (i >= camera_time_set.size())?34:camera_time_set[i]; 
      if(time_elapsed < 20 ) time_elapsed = 34; 
      else if(time_elapsed > 100 && time_elapsed < 150 ) time_elapsed *= 2; 
      else if(time_elapsed >= 150) time_elapsed *= 1.5; 
      // time_elapsed = 34; 
      imu_next_id += (int)(((float)time_elapsed)/10.+0.5); // 10 ms/frame

      // show this image 
      cv::imshow("intensity_img", i_img); 
      cv::waitKey(20);
      // construct node 
      CCameraNode* pNewNode = new CCameraNode();
      spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 

      // add node into graph 
      ADD_RET add_ret = gt_graph.addNode(pNewNode); 

      // if(gt_graph.addNode(pNewNode))
      if(add_ret == SUCC_KF)
      {
        ROS_WARN("%s succeed to add this node %d , with graph_id %d", __FILE__, pNewNode->m_seq_id, pNewNode->m_id);
        cur_pose_has_been_added = true; 
        cur_node_id = gt_graph.camnodeSize() - 1; 
      }
      else if(add_ret == FAIL_NOT_KF) // succeed, but due to KF limitation, discard this node 
      {
        // ROS_INFO("%s failed to add this node %d ", __FILE__, pNewNode->m_seq_id);
        delete pNewNode; 
        cur_pose_has_been_added = false; 
        cur_node_id =  -1; // gt_graph.camnodeSize(); 
      }else if(add_ret == FAIL_KF) // fail, often due to the featureless area 
      {
        // TODO: more suitable strategy to handle this case  
        // use fake odometry to avoids failure 
        // gt_graph.fakeOdoNode(pNewNode);
        cur_pose_has_been_added = false;
        cur_node_id = gt_graph.camnodeSize(); 
      }

      if(cur_node_id > 0) // not the first node 
      {
          // call integration before imu_factor to be constructed 
          curr_state = imu_measure->predictNext(imu_next_id); 
          // add imu factor 
          PreintegratedCombinedMeasurements* preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_measure->mp_combined_pre_imu); 
          CombinedImuFactor imu_factor(X(cur_node_id-1), V(cur_node_id-1), 
                                      X(cur_node_id),    V(cur_node_id), 
                                      B(cur_node_id-1), B(cur_node_id), *(preint_imu_combined));

          // cout <<" measure_matrix "<<endl<<preint_imu_combined->preintMeasCov(); 
          gt_graph.mp_fac_graph->add(imu_factor); 
          gt_graph.addToGTSAM(curr_state, cur_node_id, !cur_pose_has_been_added); 
    
          // Eigen::VectorXd err9  = imu_factor.unwhitenedError(*(gt_graph.mp_node_values));
          // cout <<" err9 "<<err9<<endl;
          // double err_C = (noiseModel::Gaussian::Covariance(preint_imu_combined->preintMeasCov()))->distance(err9); 
          // double err_C = C->distance(err9); 
          // ROS_ERROR("imu_factor has error_C %lf", err_C); 
          // cout<<" actual noise model sigmas: "<<endl << (noiseModel::Gaussian::Covariance(preint_imu_combined->preintMeasCov()))->sigmas();
          // double error = imu_factor.error(*(gt_graph.mp_node_values));
          // ROS_ERROR("imu_factor has errro: %lf", error); 
         
          if(!cur_pose_has_been_added)
          {
            gt_graph.m_graph_map[pNewNode->m_id] = pNewNode; 
          }
          
          imu_measure->resetPreintegrationAndBias(); 
          imu_measure->setState(curr_state); 
          
          // ROS_WARN("add imu factor %d graph error: %lf", cur_node_id, gt_graph.error());
      }

      if( CGTParams::Instance()->m_optimize_step >0 && (gt_graph.camnodeSize()%CGTParams::Instance()->m_optimize_step == 0))
      {
          gt_graph.optimizeGraph(); 
          if(cur_node_id > 0)
          {
            // TODO: update bias for MEMS? must be done           
            Values* pv = gt_graph.mp_node_values;
            curr_state = NavState((*pv).at<Pose3>(X(cur_node_id)), (*pv).at<Vector3>(V(cur_node_id))); 
            curr_bias = (*pv).at<imuBias::ConstantBias>(B(cur_node_id)); 
            // imu_measure->resetPreintegrationAndBias(curr_bias);
            imu_measure->setState(curr_state);
          }
      }

    }else{
      ROS_ERROR("%s failed to load sr_data %s", __FILE__, ss.str().c_str());
        break; 
    }
  }
  
  // dump the graph structure before optimization
  gt_graph.writeGTSAM("gtsam_structure.log");

  // compare error before optimize and after optimize 
  if(CGTParams::Instance()->m_optimize_step > 0)
  {
    // gt_graph.writeG2O("before_opt.g2o");
    ROS_INFO("before optimization error is %lf", gt_graph.error());
    gt_graph.optimizeGraph(); 
    ROS_INFO("after optimization error is %lf", gt_graph.error()); 
    gt_graph.writeG2O("./imu_fuse_mems/gtsam_result_opt.g2o");
  }
  size_t found = g_file_dir.find_last_of("/\\");
  string dataname = g_file_dir.substr(found+1) + "_trajectory.log"; 
  dataname = CGTParams::Instance()->m_output_dir + "/" + dataname; 
  gt_graph.writeTrajectory(dataname.c_str());

  return ;
}

void init_parameters()
{
  ros::NodeHandle np("~"); 
  // np.setParam("sr_data_file_dir", "/media/david/work/work/data/SLAM/SR4000/dataset_82"); 
  // np.setParam("sr_start_frame", 1); 
  // np.setParam("sr_end_frame", 2270);
  // np.setParam("sr_new_file_version", false);

  // nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  np.param("sr_data_file_dir", g_file_dir, string("/home/davidz/work/data/SwissRanger4000/exp/dataset_82")); // default parameters 
  np.param("sr_data_prefix", g_file_pre, string("d1")); 
  np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_f_end, 500); 


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

  
  np.param("imu_file_dir", g_imu_data_dir, string("")); 
  np.param("camera_syn_data", g_syn_camera_imu, string(""));
  np.param("imu_id_first_cam", g_imu_id_first_camera, 50);

}


vector<int> load_syn_camera_time() // load time sequence for camera data  
{
  vector<int> times; 
  ifstream inf(g_syn_camera_imu.c_str());
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open syn_camera_imu file %s", g_syn_camera_imu.c_str()); 
    return times; 
  }
  
  int t; 
  while(!inf.eof())
  {
    inf >> t; 
    times.push_back(t); 
  }
  return times; 
}
