/*
 * test gtsam graph slam in offline mode
 *
 * */

#include <ros/ros.h> 
#include <sstream>
#include <string>
#include "SR_reader_cv.h"
#include "sparse_feature_vo.h"
#include "camera_node.h"
#include "cam_model.h"
#include "g2o_graph.h"
#include "g2o_parameter.h"

void init_parameters(); 
void test_with_sr4k(); 

string g_file_dir; 
string g_file_pre; 
string g_file_suf;
string g_data_name;
int g_f_start; 
int g_f_end;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_g2o_with_sr4k"); 
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
  CGraphG2O g2o_graph; 
  g2o_graph.setWorld2Original(D2R(CG2OParams::Instance()->m_initial_pitch));
  
  // extract intensity and depth img for each sr_data 
  cv::Mat i_img, d_img; // 

  for(int i=g_f_start; i<g_f_end; ++i)
  {
    // get current img 
    stringstream ss; 
    ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<i<<"."<<g_file_suf; 

    // while(r4k.getCurrentFrameCV(i_img, d_img))  // get current img 
    if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
    {
      cv::imshow("intensity_img", i_img); 
      cv::waitKey(20);
      // construct node 
      CCameraNode* pNewNode = new CCameraNode();
      spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 

      // add node into graph 
      ADD_RET add_ret = g2o_graph.addNode(pNewNode); 
      // if(g2o_graph.addNode(pNewNode))
      if(add_ret == SUCC_KF)
      {
        ROS_WARN("%s succeed to add this node %d , with graph_id %d", __FILE__, pNewNode->m_seq_id, pNewNode->m_id);
        if( CG2OParams::Instance()->m_optimize_step >0 && (g2o_graph.camnodeSize()%CG2OParams::Instance()->m_optimize_step == 0))
        {
          g2o_graph.optimizeGraph(); 
        }
      }
      else if(add_ret == FAIL_NOT_KF) // succeed, but due to KF limitation, discard this node 
      {
        ROS_INFO("%s failed to add this node %d ", __FILE__, pNewNode->m_seq_id);
        delete pNewNode; 
      }else if(add_ret == FAIL_KF) // fail, often due to the featureless area 
      {
        // TODO: more suitable strategy to handle this case  
        // use fake odometry to avoids failure 
        g2o_graph.fakeOdoNode(pNewNode); 
        
      }
    }else{
      ROS_ERROR("%s failed to load sr_data %s", __FILE__, ss.str().c_str());
        break; 
    }
  }

  string dataname; 

  // compare error before optimize and after optimize 
  if(CG2OParams::Instance()->m_optimize_step > 0)
  {

    dataname = CG2OParams::Instance()->m_output_dir + "/" + g_data_name + "_vo_before_g2o.ply"; 
    g2o_graph.trajectoryPLY(dataname, BLUE); 

    // g2o_graph.writeG2O("before_opt.g2o");
    ROS_INFO("before optimization error is %lf", g2o_graph.error());
    g2o_graph.optimizeGraph(); 
    ROS_INFO("after optimization error is %lf", g2o_graph.error()); 
 
    // size_t found = g_file_dir.find_last_of("/\\");
    // string dataname = g_file_dir.substr(found+1) + "_trajectory.g2o"; 
    //  dataname = CG2OParams::Instance()->m_output_dir + "/" + dataname;    
  }
    dataname = CG2OParams::Instance()->m_output_dir + "/" + g_data_name + "_vo_after_g2o.ply"; 
    g2o_graph.trajectoryPLY(dataname, RED);

    dataname = CG2OParams::Instance()->m_output_dir + "/" + g_data_name + "_vo_after_trajectory_g2o.log"; 
    // g2o_graph.writeG2O("./imu_fuse_mems/g2o_result_opt.g2o");
    g2o_graph.writeTrajectory(dataname.c_str());

  // size_t found = g_file_dir.find_last_of("/\\");
  // string dataname = g_file_dir.substr(found+1) + "_trajectory.log"; 
  // dataname = CG2OParams::Instance()->m_output_dir + "/" + dataname; 
  // g2o_graph.writeTrajectory(dataname.c_str());

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
  np.param("sr_data_name", g_data_name, g_data_name);

  CG2OParams* p = CG2OParams::Instance(); 
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
}


