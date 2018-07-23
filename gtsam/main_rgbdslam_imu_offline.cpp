/*
 * July 20 2018, He Zhang, hzhang8@vcu.edu 
 * 
 * loosely couple rgbd + imu in gtsam framework 
 *
 * */

#include <ros/ros.h> 
#include <sstream>
#include <string>
// #include "SR_reader_cv.h"
#include "rs_r200_wrapper.h"
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
void rgbdslam_imu_offline(); 
bool read_dataset(string data_dir, vector<double>& time_vf, vector<string>& rgb_vf, vector<string>& dpt_vf); 

string g_file_dir; 
string g_file_pre; 
string g_file_suf;
string g_data_name;

int g_c;  // trajectory color
int g_f_start = 1; 
// int g_f_end;

string g_imu_file; // imu measurement file 
string g_img_time_file; // sr timestamp file, which is used to synchronize between camera data and imu data 
string g_vro_results_file; // where the vro results are stored 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "main_rgbdslam_imu_offline"); 
    ros::NodeHandle n; 

    ROS_INFO("at line %s before main_rgbdslam_imu_offline", __FILE__);

    rgbdslam_imu_offline(); 

    return 0; 
}

void rgbdslam_imu_offline()
{
  // parameters 
  init_parameters(); 

  // read data 
  vector<string> rgb_vf; 
  vector<string> dpt_vf; 
  vector<double> time_vf;
  if(!read_dataset(g_file_dir, time_vf, rgb_vf, dpt_vf))
  {
      cout<<"main_rgbdslam_imu_offline.cpp: failed to load data from dir: "<<g_file_dir<<endl; 
      return ; 
  }else{
      cout<<"main_rgbdslam_imu_offline.cpp: succeed to load "<<rgb_vf.size()<<" image."<<endl; 
  }

  // camera 
  // CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  CamModel rs435(617.306, 617.714, 326.245, 239.974); 
  CCameraNode::set_cam_cov(rs435);
  float depth_scale = 0.001;
  rs435.setDepthScale(depth_scale); 
  CamModel::updategCamModel(rs435); 
  CSparseFeatureVO spr_vo(rs435);

  // cam data io 
  // CSReadCV r4k; 
  CRSR200Wrapper cam; 

  // set graph strcuture 
  CGraphGT gt_graph; 
   // VRO result 
  gt_graph.readVRORecord(g_vro_results_file); 
  // gt_graph.setWorld2Original(D2R(CGTParams::Instance()->m_initial_pitch));
  // gt_graph.setCamera2IMU(0);  // TODO: add Tu2c 
  gt_graph.setCamera2IMUTranslation(-0.01, 0, 0.025); 

  // extract intensity and depth img for each data
  cv::Mat i_img, d_img; // 

  // imu parameter
  gtsam::imuBias::ConstantBias prior_bias; 
  double dt = 0.005; // 200 hz
  CImuVn100* imu = new CImuVn100(dt, prior_bias); 
  if(!imu->readImuData(g_imu_file))
  {
    ROS_ERROR("failed to load imu data from file %s", g_imu_file.c_str()); 
    return ; 
  }

  ROS_INFO("start the first node process");
  // add first node 
  string rgb_file, dpt_file; 
  rgb_file = g_file_dir + "/" + rgb_vf[g_f_start-1]; 
  dpt_file = g_file_dir + "/" + dpt_vf[g_f_start-1]; 
  if(!cam.readOneFrameCV(rgb_file, dpt_file, i_img, d_img))
  {
    cout<<"main_rgbdslam_offline.cpp: failed to load rgb_file: "<<rgb_file<<endl; 
    return ; 
  }

  CCameraNode* pNewNode = new CCameraNode();
  spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
  pNewNode->m_seq_id = g_f_start;
  gt_graph.firstNode(pNewNode, false);
  imu->setStartPoint(time_vf[g_f_start-1]);  // first measurement 
  
  // initialization from imu measurement 
  double ax, ay, az; 
  imu->getNormalizedAcc(ax, ay, az); 
  gt_graph.initFromImu(ax, ay, az); 

  // record consequent NavState 
  NavState pre_state, cur_state; 
  cv::Mat pre_rgb, pre_dpt; 

  // traversely visit the vro matching results 
  int cur_frame_id = g_f_start; 
  int cur_imu_id = cur_frame_id; 
  int cur_node_id = cur_imu_id; 
  
  ROS_INFO("at line %d start to read vro results", __LINE__);

  for(int i=0; i<gt_graph.mv_vro_res.size() && ros::ok(); i++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[i]; 
    
    if(pm->edge.id2 <= g_f_start) continue; // skip edges

    if(pm->edge.id2 > cur_frame_id) // a new frame is aligned, an incremental edge is added 
    {
      cur_imu_id = pm->edge.id2;
      // ROS_INFO("process sequence node id : %d", cur_imu_id);
      // stringstream ss; 
      // ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<pm->edge.id2<<"."<<g_file_suf;  // setw(4)
      rgb_file = g_file_dir + "/" + rgb_vf[pm->edge.id2-1]; 
      dpt_file = g_file_dir + "/" + dpt_vf[pm->edge.id2-1]; 

      if(cam.readOneFrameCV(rgb_file, dpt_file, i_img, d_img))
      // if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
      {
        CCameraNode* pNewNode = new CCameraNode();
        spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
        bool valid_match = gt_graph.addNodeOffline(pNewNode, pm);
        if(!valid_match) // failed to add vo factor, so the pose node has not been added,        
        {
          // manually add node into graph 
          gt_graph.m_graph_map[pNewNode->m_id] = pNewNode; 
        }
        
        // imu preintegration,  add imu factor between these two nodes 
        NavState cur_p;
        bool imu_available = imu->predictNextFlag(time_vf[cur_imu_id-1], cur_p);
        PreintegratedCombinedMeasurements* preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu->mp_combined_pre_imu); 

        if(imu_available)
        {
          // add imu measurement into graph 
          cur_node_id = pNewNode->m_id; 
                CombinedImuFactor imu_factor(X(cur_node_id-1), V(cur_node_id-1), 
              X(cur_node_id),    V(cur_node_id), 
              B(cur_node_id-1), B(cur_node_id), *(preint_imu_combined));

          gt_graph.mp_fac_graph->add(imu_factor); 
          gt_graph.mp_new_fac->add(imu_factor); 
          ROS_INFO("INSERT Node id %d , with seq_id %d", cur_node_id, pNewNode->m_seq_id);
          gt_graph.addToGTSAM(cur_p, cur_node_id, !valid_match); 
          Eigen::MatrixXd Cov = preint_imu_combined->preintMeasCov(); 
       }
         
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
      int j = i; 
      // ROS_INFO("First loop edge pm->edge.id2: %d id1: %d i= %d cur_frame_id = %d", pm->edge.id2, pm->edge.id1, i, cur_frame_id);
      while(j < gt_graph.mv_vro_res.size())
      {
        pm = gt_graph.mv_vro_res[j]; 
        if(pm->edge.id2 > cur_frame_id)
        {
          break; 
        }

        // if(node_removed_by_chi2 == false) // only add loop edge when the new node is not removed
           gt_graph.addEdgeOffline(pm); 
        j++; 
      }
      
      i = j - 1; // update i
      
      // gt_graph.mp_new_fac->print("loop fac: ");
    }

    // optimize graph 
    gt_graph.optimizeGraphIncremental(); 
    // gt_graph.optimizeGraphBatch();

    // reset imu bias parameters 
    imu->resetPreintegrationAndBias(gt_graph.mp_node_values->at<imuBias::ConstantBias>(B(cur_node_id))); 
    pre_state = NavState(gt_graph.mp_node_values->at<Pose3>(X(cur_node_id)), gt_graph.mp_node_values->at<Vector3>(V(cur_node_id))); 
    imu->setState(pre_state); 

    // gt_graph.mp_fac_graph->print("current graph: "); 

    // print graph to see what's going on 
    // gt_graph.mp_node_values->print("current estimate : ");
    // if(i>=3) break; 
  }
 
  string mode = "_vio"; 
  ROS_INFO("before optimization error is %lf", gt_graph.error());
  string dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + mode + ".ply";
  gt_graph.trajectoryPLY(dataname, static_cast<CG::COLOR>(g_c%7)); // CG::BLUE 
  // gt_graph.optimizeGraphBatch(); 

  // ROS_INFO("after optimization error is %lf", gt_graph.error()); 
  // dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + mode + "_after.ply"; 
  // gt_graph.trajectoryPLY(dataname, CG::RED); 

  // record trajectory for later mapping 
  dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + mode + "_trajectory.log"; 
  gt_graph.writeTrajectory(dataname.c_str());

  // output final result 
  // size_t found = g_file_dir.find_last_of("/\\");
  // string dataname = g_file_dir.substr(found+1) + "_trajectory.log"; 
  // dataname = CGTParams::Instance()->m_output_dir + "/" + dataname; 
  // gt_graph.writeTrajectory(dataname.c_str());

  return ;
}

void init_parameters()
{
  ros::NodeHandle np("~"); 
  np.param("data_file_dir", g_file_dir, string("")); 
  np.param("data_name", g_data_name, string("default")); 
  np.param("imu_file", g_imu_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/imu_v100.log"));
  np.param("img_time_file", g_img_time_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/sr4k/timestamp.log")); 
  np.param("vro_results_file", g_vro_results_file, string("/home/david/.ros/vro_results/etas_f2_vro_results.log")); 
//  np.param("chi2_for_vro", g_chi2_test, g_chi2_test); 
//  np.param("plane_aided", g_plane_factor, true);
//  np.param("view_plane", g_view_plane, g_view_plane); 

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
  // np.param("gt_initial_pitch",     p->m_initial_pitch, -14.7);
  np.param("gt_initial_pitch",     p->m_initial_pitch, 0.);
  np.param("vo_result_file",       p->m_vro_result, std::string("vro_results.log"));
  np.param("trajectory_color", g_c, 1);
}

bool read_dataset(string data_dir, vector<double>& time_vf, vector<string>& rgb_vf, vector<string>& dpt_vf)
{
    string log_file = data_dir + "/timestamp.txt"; 
    ifstream inf(log_file.c_str()); 
    if(!inf.is_open())
    {
	cout <<"main_rgbdslam_offline.cpp: failed to open file: "<<log_file<<endl; 
	return false; 
    }

    rgb_vf.clear(); 
    dpt_vf.clear(); 
    
    char buf[4096];
    while(!inf.eof())
    {
	string s; 
	getline(inf, s); 
	if(!s.empty())
	{
	    double t;
	    string rgb_f, dpt_f; 
	    stringstream ss; 
	    ss << s; 
	    ss >> t; 
	    ss >> rgb_f; 
	    ss >> t; 
	    ss >> dpt_f; 
	    rgb_vf.push_back(rgb_f);
	    dpt_vf.push_back(dpt_f); 
	    time_vf.push_back(t); 
	}
    }
    return rgb_vf.size() > 0; 
}

