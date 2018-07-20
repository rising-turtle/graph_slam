/*
    July 19 2018, He Zhang, hzhang8@vcu.edu 
    
    offline style - pipeline of rgbdslam, records of vro's result enabled

*/

#include <ros/ros.h> 
#include <sstream>
#include <string>
// #include "SR_reader_cv.h"
#include "rs_r200_wrapper.h"
#include "sparse_feature_vo.h"
#include "camera_node_pnp.h"
#include "cam_model.h"
#include "gtsam_graph.h"
#include "gt_parameter.h"

using namespace std; 
using namespace CG; 

void init_parameters(); 
void offline_rgbdslam(); 
bool read_dataset(string data_dir, vector<string>& rgb_vf, vector<string>& dpt_vf); 

string g_file_dir; 
string g_data_name; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "main_rgbdslam_offline"); 
  ros::NodeHandle n; 

  offline_rgbdslam(); 

  return 0; 
}

void offline_rgbdslam()
{
    // set parameters
    init_parameters();

    // read data 
    vector<string> rgb_vf; 
    vector<string> dpt_vf; 
    if(!read_dataset(g_file_dir, rgb_vf, dpt_vf))
    {
	cout<<"main_rgbdslam_offline.cpp: failed to load data from dir: "<<g_file_dir<<endl; 
	return ; 
    }else{
	cout<<"main_rgbdslam_offline.cpp: succeed to load "<<rgb_vf.size()<<" image."<<endl; 
    }

    // camera interface 
    // CSReadCV r4k; 
    CRSR200Wrapper cam; 

    // set camera model and sparse feature module 
    // CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
    CamModel rs435(617.306, 617.714, 326.245, 239.974);
    // CCameraNode::set_cam_cov(sr4k); 
    CCameraNode::set_cam_cov(rs435); 

    float depth_scale = 0.001;
    // CSparseFeatureVO spr_vo(sr4k);
    CSparseFeatureVO spr_vo(rs435); 

    // set graph strcuture 
    CGraphGT gt_graph; 
    // gt_graph.setWorld2Original(D2R(CGTParams::Instance()->m_initial_pitch));
    // gt_graph.setCamera2IMU(0); 

    // extract intensity and depth img for each sr_data 
    cv::Mat i_img, d_img; // 

    for(int i=0; i<rgb_vf.size(); ++i)
    {
	// while(r4k.getCurrentFrameCV(i_img, d_img))  // get current img 
	string rgb_file = g_file_dir + "/" + rgb_vf[i]; 
	string dpt_file = g_file_dir + "/" + dpt_vf[i]; 
	// if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
	if(cam.readOneFrameCV(rgb_file, dpt_file, i_img, d_img))
	{
	    cv::imshow("intensity_img", i_img); 
	    cv::waitKey(20);
	    // construct node 
	    CCameraNode* pNewNode = new CCameraNode();
	    // CCameraNode* pNewNode = new CCameraNodePnP(); 
	    spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 

	    // add node into graph 
	    ADD_RET add_ret = gt_graph.addNode(pNewNode); 
	    // if(gt_graph.addNode(pNewNode))
	    if(add_ret == SUCC_KF)
	    {
		ROS_WARN("%s succeed to add this node %d , with graph_id %d", __FILE__, pNewNode->m_seq_id, pNewNode->m_id);
		if( CGTParams::Instance()->m_optimize_step >0 && (gt_graph.camnodeSize()%CGTParams::Instance()->m_optimize_step == 0))
		{
		    gt_graph.optimizeGraph(); 
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
		gt_graph.fakeOdoNode(pNewNode); 

	    }
	}else{
	    ROS_ERROR("%s failed to load sr_data %s", __FILE__, rgb_file.c_str());
	    break; 
	}
    }
    string dataname ; 
    // compare error before optimize and after optimize 
    if(CGTParams::Instance()->m_optimize_step > 0)
    {
	// gt_graph.writeG2O("before_opt.g2o");
	ROS_INFO("before optimization error is %lf", gt_graph.error());
	dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + "_vo_before.ply"; 
	gt_graph.trajectoryPLY(dataname, BLUE); 
	gt_graph.optimizeGraph(); 
	ROS_INFO("after optimization error is %lf", gt_graph.error()); 
	// gt_graph.writeG2O("gtsam_result_opt.g2o");
    }
    // size_t found = g_file_dir.find_last_of("/\\");
    //  string dataname = g_file_dir.substr(found+1) + "_trajectory.log"; 

    dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + "_vo_after.ply"; 
    gt_graph.trajectoryPLY(dataname, RED); 

    dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + "_vo_after_trajectory.log"; 
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
  np.param("data_file_dir", g_file_dir, string("")); 

  /*
  np.param("sr_data_file_dir", g_file_dir, string("/home/davidz/work/data/SwissRanger4000/exp/dataset_82")); // default parameters 
  np.param("sr_data_prefix", g_file_pre, string("d1")); 
  np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_f_end, 500); */
  np.param("sr_data_name", g_data_name, string("default"));

  CGTParams* p = CGTParams::Instance(); 
  p->m_small_translation = 0.04; 
  p->m_small_rotation = 3; 
  p->m_lookback_nodes = 5; 
  p->m_optimize_step = 10; 
  p->m_record_vro_results = true; 
  int ransac_k = 5000; 
  np.param("vo_small_translation", p->m_small_translation, 0.04); 
  np.param("vo_small_rotation"  ,  p->m_small_rotation, 3.); 
  np.param("vo_ransac_time"     ,  ransac_k, ransac_k); 
  np.param("vo_result_file" ,      p->m_vro_result, std::string("vro_results.log"));
  np.param("gt_optimize_step",     p->m_optimize_step, 10); 
  np.param("gt_lookback_nodes",    p->m_lookback_nodes, 5);
  np.param("gt_output_dir",        p->m_output_dir, std::string("./"));
  // np.param("gt_initial_pitch",     p->m_initial_pitch, -14.7);
  np.param("gt_initial_pitch",     p->m_initial_pitch, 0.0);

  // parameters configure 
  CParams* pP = CParams::Instance(); 
  pP->m_feature_detector_type = "SIFT";  // SIFTGPU 
  pP->m_feature_descriptor_type = "SIFT";  // SIFTGPU
  pP->m_feature_match_type = "FLANN"; 
  pP->m_nn_distance_ratio = 0.5; // 0.95 for SIFTGPU, 0.5-0.7 for SIFT 
  pP->m_max_dist_for_inliers = 0.05;  
  pP->m_max_num_features = 500; 
  pP->m_ransac_iterations = ransac_k; // 
  pP->m_min_matches = 12; // this parameter is important 
}


bool read_dataset(string data_dir, vector<string>& rgb_vf, vector<string>& dpt_vf)
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
	    string t, rgb_f, dpt_f; 
	    stringstream ss; 
	    ss << s; 
	    ss >> t; 
	    ss >> rgb_f; 
	    ss >> t; 
	    ss >> dpt_f; 
	    rgb_vf.push_back(rgb_f);
	    dpt_vf.push_back(dpt_f); 
	}
    }
    return rgb_vf.size() > 0; 
}

