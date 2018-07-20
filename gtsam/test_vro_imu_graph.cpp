/*
 * test gtsam graph slam in offline mode
 * 
 * using vro and imu measurement 
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

int g_c;  // trajectory color

bool g_chi2_test = false;   // whether to use Chi2 test to delete VRO edges 
bool g_plane_factor = false; // whether to add plane factors into graph 
bool g_view_plane = false;  // whether to view the result of plane propagation 
int g_f_start; 
int g_f_end;

string g_imu_file; // imu measurement file 
string g_img_time_file; // sr timestamp file, which is used to synchronize between camera data and imu data 
string g_vro_results_file; // where the vro results are stored 

bool loadImgTime(map<int, double>&); // load synchronized time stamps

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

  // IMU files
  map<int, double> img_times; 
  if(!loadImgTime(img_times))
  {
    ROS_ERROR("failed to read time file %s", g_img_time_file.c_str());
    return ; 
  }
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
  stringstream ss; 
  ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<g_f_start<<"."<<g_file_suf;  // setw(4)
  r4k.readOneFrameCV(ss.str(), i_img, d_img);

  CCameraNode* pNewNode = new CCameraNode();
  spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
  pNewNode->m_seq_id = g_f_start;
  gt_graph.firstNode(pNewNode, false);
  imu->setStartPoint(img_times[g_f_start]);  // first measurement 

  // record consequent NavState 
  NavState pre_state, cur_state; 
  cv::Mat pre_rgb, pre_dpt; 

  if(g_plane_factor) // use plane factor 
  {
    CPlaneNode* p1 = new CPlaneNode(); 
    int ret = p1->extractPlanes(i_img, d_img, &sr4k); 
    if(ret == 0)
    {
       delete p1; p1 = NULL;
    }
    gt_graph.firstPlaneNode(p1); 
    pre_rgb = i_img.clone(); 
    pre_dpt = d_img.clone(); 
  }

  // traversely visit the vro matching results 
  int cur_frame_id = g_f_start; 
  int cur_imu_id = cur_frame_id; 
  int cur_node_id = cur_imu_id; 
  
  ROS_INFO("at line %d start to read vro results", __LINE__);

  // if this node is removed due to Chi2 check, then the related local loop edges are removed also
  bool node_removed_by_chi2 = false; 

  for(int i=0; i<gt_graph.mv_vro_res.size() && i < g_f_end && ros::ok(); i++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[i]; 
    
    if(pm->edge.id2 <= g_f_start) continue; // skip edges

    if(pm->edge.id2 > cur_frame_id) // a new frame is aligned, an incremental edge is added 
    {
      cur_imu_id = pm->edge.id2;
      // ROS_INFO("process sequence node id : %d", cur_imu_id);
      stringstream ss; 
      ss<<g_file_dir<<"/"<<g_file_pre<<"_"<<setfill('0')<<setw(7)<<pm->edge.id2<<"."<<g_file_suf;  // setw(4)

      if(r4k.readOneFrameCV(ss.str(), i_img, d_img))
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
        bool imu_available = imu->predictNextFlag(img_times[cur_imu_id], cur_p);
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
        // whether add plane node  
        if(g_plane_factor)
        {
          int pre_plane_num = gt_graph.m_plane_landmark_id; 
          if(!valid_match && imu_available)
          {
            double trace_SIMU; 
            Eigen::MatrixXd Cov = preint_imu_combined->preintMeasCov(); 
            // Eigen::MatrixXd Information = Cov.inverse(); 
            Matrix6 S_pose = Cov.block<6,6>(0,0); 
            trace_SIMU = S_pose(0,0) + S_pose(1,1) + S_pose(2,2); 

            int pre_node_id = pNewNode->m_id - 1; 
            // only add plane landmark when VRO fails 
            // previous plane Node 
            CPlaneNode * prePNode = gt_graph.mv_plane_nodes[pre_node_id];
            ROS_INFO("%d get preNode = %ld", __LINE__, prePNode); 
            
            if(prePNode == NULL)
            {
              prePNode = new CPlaneNode(); 
              ROS_INFO("%d get preNode = NULL, try to extractPlanes", __LINE__); 
              int ret = prePNode->extractPlanes(pre_rgb, pre_dpt, &sr4k); 
              if(ret == 0) // hopeless case
              {
                ROS_INFO("%d no plane detected, delete it", __LINE__); 
                delete prePNode; prePNode = NULL;
              }else{
                gt_graph.planeNodeAssociation(pNewNode->m_id-1, prePNode, trace_SIMU); 
              }
            }

            // CPlaneNode* prePNode = gt_graph.mv_plane_nodes[pNewNode->m_id-1];
            // current plane Node 
            CPlaneNode * curPNode = NULL; 
            if(prePNode != NULL)
            {
              ROS_INFO("%d pre planeNode is not NULL, try to predictPlaneNode", __LINE__); 
              Pose3 pre_p = gt_graph.mp_node_values->at<Pose3>(X(pNewNode->m_id-1));
              Pose3 inc_p = pre_p.transform_pose_to(cur_p.pose()); 
                    curPNode = gt_graph.predictPlaneNode(prePNode, i_img, d_img, &inc_p, S_pose);
            }else
            {
              curPNode = new CPlaneNode();
              ROS_INFO("%d pre planeNode is NULL, try to extractPlanes for curPNode", __LINE__);
              int ret = curPNode->extractPlanes(i_img, d_img, &sr4k); 
              if(ret == 0)
              {
                ROS_INFO("%d cur planeNode is NULL, delete it", __LINE__);
                delete curPNode; curPNode = NULL; 
              }
            }

            // add plane factors 
            bool b_has_new_plane = false; 
            if(curPNode != NULL)
            {
              for(int i=0; i<curPNode->mv_planes.size(); i++)
              {
                ROS_INFO("%d cur planeNode has planes, add plane %i", __LINE__, i);
                // ROS_INFO("curPlane has plane %d landmark_id %d", i, curPlane->mv_landmark_id[i]);
                if(curPNode->mv_landmark_id[i] < 0) 
                {
                  b_has_new_plane = true; 
                  break; 
                }
                gt_graph.addPlaneFactor(curPNode->mv_planes[i], pNewNode->m_id, curPNode->mv_landmark_id[i]); 
              }
              if(curPNode->mv_planes.size() == 0)
              {
                ROS_INFO("curPlane has no planes, delete it"); 
                delete curPNode;  curPNode = NULL; 
              }
            }

            // plane association 
            if(b_has_new_plane)
            {
               ROS_INFO("curPlane has %d planes try to associate them ", curPNode->mv_planes.size());
               gt_graph.planeNodeAssociation(pNewNode->m_id, curPNode, trace_SIMU); 
            }

            gt_graph.mv_plane_nodes[pNewNode->m_id] = curPNode; // no plane is needed

            // view the plane 
            if(gt_graph.m_plane_landmark_id > pre_plane_num && g_view_plane) // some new plane is added, show it 
            {
                ROS_INFO("curPlane add planes, view them ");

                cv::Mat rgb = cv::Mat(i_img.size(), CV_8UC3); 
                copyMat(i_img, rgb);
                int c = GREEN; 
                for(int i=curPNode->mv_planes.size()-1; i>=0; i--)
                {
                  markColor(rgb, curPNode->mv_indices[i], static_cast<COLOR>(c++%5)); 
                }
                
                stringstream ss; ss << "node at: "<<pNewNode->m_seq_id<<endl;
                cv::namedWindow(ss.str().c_str()); 
                cv::imshow(ss.str().c_str(), rgb); 
                cv::waitKey(0); 
                cv::destroyWindow(ss.str().c_str());
                ROS_INFO("after view them ");
            }

          }  else{
            // TODO: 
             gt_graph.mv_plane_nodes[pNewNode->m_id] = NULL; 
           } 
          
          pre_rgb = i_img.clone(); 
          pre_dpt = d_img.clone(); 
          
        }
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

        if(node_removed_by_chi2 == false) // only add loop edge when the new node is not removed
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
  if(g_plane_factor) mode="_pvio"; 
  // gt_graph.writeG2O("before_opt.g2o");
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

void test_read_vro()
{
  std::string fname = "/home/david/.ros/vro_results/etas_f5_vro_results.log"; 
  
  // set camera model and sparse feature module 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  CCameraNode::set_cam_cov(sr4k); 

  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;
  CSparseFeatureVO spr_vo(sr4k);
 
  CGraphGT g; 
  
  // VRO result 
  g.readVRORecord(fname); 
  
  ROS_INFO("succeed to read %d vro results!", g.mv_vro_res.size());

  // for(int i=0; i<g.mv_vro_res.size(); i++)
  for(int i=0; i<100; i++)
  {
     // g.recordVROResult(*(g.mv_vro_res[i]));
     g.printVROResult(std::cout, *(g.mv_vro_res[i]));
  }

  Eigen::Matrix4d m = g.mv_vro_res[3]->edge.transform.matrix();
  gtsam::Pose3 p(m); 
  gtsam::Vector6 r = Pose3::ChartAtOrigin::Local(p); 
  gtsam::Pose3 p2 = gtsam::Pose3::ChartAtOrigin::Retract(r); 
  Eigen::Matrix4d m2 = p2.matrix(); 

  p.print("p1 : "); 
  p2.print("p2 : ");

  cout <<"m1 : "<<endl<<m<<endl; 
  cout <<"m2 : "<<endl<<m2<<endl;

  return ;
}

bool loadImgTime(map<int, double>& mt) // load synchronized time stamps
{
  ifstream inf(g_img_time_file);
  if(!inf.is_open())
  {
    ROS_ERROR("%d failed to load camera timestamp %s", __LINE__, g_img_time_file.c_str()); 
    return false; 
  }
  int img_id; 
  double img_timestamp; 
  while(!inf.eof())
  { 
    inf>>img_id>>img_timestamp; 
    // ROS_INFO("img_id %d time %lf", img_id, img_timestamp);
    mt[img_id] = img_timestamp; 
  }
  return mt.size() > 0; 
}


void copyMat(cv::Mat& f, cv::Mat& t)
{
  if(f.type() == t.type() )
  {
    t = f.clone();
    return ;
  }

  int color_index, grey_index; 
  int rgb_size = 3; 
  int grey_size =1; 
  if(f.type() == CV_8UC1 && t.type() == CV_8UC3)
  {
    for(int r=0; r<t.rows; r++)
    for(int c=0; c<t.cols; c++)
    {
      color_index = (r*t.cols + c)*rgb_size; 
      grey_index = (r*t.cols + c)*grey_size; 
      t.at<uint8_t>(color_index + 2) = f.at<uint8_t>(grey_index);  // r
      t.at<uint8_t>(color_index + 1) = f.at<uint8_t>(grey_index);  // g 
      t.at<uint8_t>(color_index + 0) = f.at<uint8_t>(grey_index);  // b
    }
  }else
  {
    ROS_ERROR("%s TODO: copyMat with other Mat types", __FILE__); 
  }
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
  // np.param("sr_data_file_dir", g_file_dir, string("/home/davidz/work/data/SwissRanger4000/exp/dataset_82")); // default parameters 
  np.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f2")); // default parameters 
  np.param("sr_data_prefix", g_file_pre, string("d1")); 
  np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_f_end, 500); 
  np.param("sr_data_name", g_data_name, string("etas_f2"));

  np.param("imu_file", g_imu_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/imu_v100.log"));
  np.param("imu_time_file", g_img_time_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/sr4k/timestamp.log")); 
  np.param("vro_results_file", g_vro_results_file, string("/home/david/.ros/vro_results/etas_f2_vro_results.log")); 
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
  np.param("vo_result_file",       p->m_vro_result, std::string("vro_results.log"));
  np.param("trajectory_color", g_c, 1);

}

      /*
          // fake a identity node 
          cur_node_id = pNewNode->m_id; 

          Pose3 new_pose = gt_graph.mp_node_values->at<Pose3>(X(cur_node_id-1));
          gt_graph.mp_node_values->insert(X(cur_node_id), new_pose);
          gt_graph.mp_new_node->insert(X(cur_node_id), new_pose);

          Eigen::Matrix<double, 6, 6> tmp = Eigen::Matrix<double, 6, 6>::Identity()*10000; 
          noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(tmp);

          Pose3 inc_pose; 
          gt_graph.mp_fac_graph->add(BetweenFactor<Pose3>(X(cur_node_id-1), X(cur_node_id), inc_pose, visual_odometry_noise)); 
          gt_graph.mp_new_fac->add(BetweenFactor<Pose3>(X(cur_node_id-1), X(cur_node_id), inc_pose, visual_odometry_noise)); 
          */  


/*
 
        CCameraNode* pNewNode = new CCameraNode();
        spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
        bool valid_match = gt_graph.addNodeOffline(pNewNode, pm);
        if(!valid_match) // failed to add vo factor, so the pose node has not been added,        
        {
          // manually add node into graph 
          gt_graph.m_graph_map[pNewNode->m_id] = pNewNode; 

          /*
          // fake a identity node 
          cur_node_id = pNewNode->m_id; 

          Pose3 new_pose = gt_graph.mp_node_values->at<Pose3>(X(cur_node_id-1));
          gt_graph.mp_node_values->insert(X(cur_node_id), new_pose);
          gt_graph.mp_new_node->insert(X(cur_node_id), new_pose);

          Eigen::Matrix<double, 6, 6> tmp = Eigen::Matrix<double, 6, 6>::Identity()*10000; 
          noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(tmp);

          Pose3 inc_pose; 
          gt_graph.mp_fac_graph->add(BetweenFactor<Pose3>(X(cur_node_id-1), X(cur_node_id), inc_pose, visual_odometry_noise)); 
          gt_graph.mp_new_fac->add(BetweenFactor<Pose3>(X(cur_node_id-1), X(cur_node_id), inc_pose, visual_odometry_noise)); 
          
        }
        
        // imu preintegration,  add imu factor between these two nodes 
        NavState cur_p = imu->predictNext(img_times[cur_imu_id]);

        // add imu measurement into graph 
        cur_node_id = pNewNode->m_id; 
        PreintegratedCombinedMeasurements* preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu->mp_combined_pre_imu); 
        CombinedImuFactor imu_factor(X(cur_node_id-1), V(cur_node_id-1), 
                                    X(cur_node_id),    V(cur_node_id), 
                            B(cur_node_id-1), B(cur_node_id), *(preint_imu_combined));
 
        gt_graph.mp_fac_graph->add(imu_factor); 
        gt_graph.mp_new_fac->add(imu_factor); 
        gt_graph.addToGTSAM(cur_p, cur_node_id, !valid_match); 
        
        // whether add plane node  
        if(g_plane_factor && !valid_match)
        {
          CPlaneNode* curPlane; 
          
          int pre_plane_num = gt_graph.m_plane_landmark_id; 

          // plane detection  
          CPlaneNode* prePlane = gt_graph.mv_plane_nodes[pNewNode->m_id-1]; 
          double trace_SIMU; 
          if(prePlane != NULL)
          {
            // 1. project the previous plane feature into the current frame 
            Pose3 pre_p = gt_graph.mp_node_values->at<Pose3>(X(pNewNode->m_id-1));
            Pose3 inc_p = pre_p.transform_pose_to(cur_p.pose()); 

            Eigen::MatrixXd Cov = preint_imu_combined->preintMeasCov(); 
            // Eigen::MatrixXd Information = Cov.inverse(); 
            Matrix6 S_pose = Cov.block<6,6>(0,0); 
            trace_SIMU = S_pose(0,0) + S_pose(1,1) + S_pose(2,2); 
            // ROS_INFO("before predictPlaneNode"); 
            curPlane = gt_graph.predictPlaneNode(prePlane, i_img, d_img, &inc_p, S_pose); 

          }else{
            // 2. detect plane itself
            curPlane = new CPlaneNode(); 
            // ROS_INFO("before extractPlanes");
            int ret = curPlane->extractPlanes(i_img, d_img, &sr4k); 
            if(ret == 0)
            {
              delete curPlane; curPlane = NULL;
            }
          }
          
          // add plane factors 
          bool b_has_new_plane = false; 
          if(curPlane != NULL)
          {
            for(int i=0; i<curPlane->mv_planes.size(); i++)
            {
              // ROS_INFO("curPlane has plane %d landmark_id %d", i, curPlane->mv_landmark_id[i]);
              if(curPlane->mv_landmark_id[i] < 0) 
              {
                b_has_new_plane = true; 
                break; 
              }
              gt_graph.addPlaneFactor(curPlane->mv_planes[i], pNewNode->m_id, curPlane->mv_landmark_id[i]); 
            }
            if(curPlane->mv_planes.size() == 0)
            {
              // ROS_INFO("curPlane has no planes, delete it"); 
              delete curPlane;  curPlane = NULL; 
            }
          }

          // plane association 
          if(b_has_new_plane)
          {
            // ROS_INFO("curPlane has %d planes", curPlane->mv_planes.size());
            gt_graph.planeNodeAssociation(pNewNode->m_id, curPlane, trace_SIMU); 
          }
          
          // ROS_INFO("after plane association");
          gt_graph.mv_plane_nodes[pNewNode->m_id] = curPlane; 
          if(curPlane == NULL)
          {
            // ROS_INFO("node %d has a null plane pointer", pNewNode->m_id); 
          }else{
            ROS_INFO("node %d has a valid plane node with %d planes", pNewNode->m_id, curPlane->mv_planes.size());
            for(int n=0; n<curPlane->mv_planes.size(); ++n)
            {
              if(curPlane->mv_landmark_id[n] < 0)
              {
                ROS_ERROR("node id: %d has a plane id = -1", pNewNode->m_id); 
              }else{
                ROS_WARN("node id: %d has a plane id = %d", pNewNode->m_id, curPlane->mv_landmark_id[n]);
              }
            }
            
            if(gt_graph.m_plane_landmark_id > pre_plane_num && g_view_plane) // some new plane is added, show it 
            {
                cv::Mat rgb = cv::Mat(i_img.size(), CV_8UC3); 
                copyMat(i_img, rgb);
                int c = GREEN; 
                for(int i=curPlane->mv_planes.size()-1; i>=0; i--)
                {
                  markColor(rgb, curPlane->mv_indices[i], static_cast<COLOR>(c++%5)); 
                }
                
                stringstream ss; ss << "node at: "<<pNewNode->m_seq_id<<endl;
                cv::namedWindow(ss.str().c_str()); 
                cv::imshow(ss.str().c_str(), rgb); 
                cv::waitKey(0); 
                cv::destroyWindow(ss.str().c_str());
            }
          }
   } // 

 * */




/*

        // chi2 test for VRO edges 
        if(g_chi2_test && valid_match) // this means that the VRO succeed, and here use chi2 to further verify the validity of this VRO range
        {
          int N = 3; 
          double CI = utils::chi2(N, 0.95);  // the dimension for rotation is three, quantile(Chi2(N), 0.95)
          
          // difference between pre_state and cur_state 
          // cur_state = cur_p; 
          // Pose3 vro_p = gt_graph.mp_node_values->at<Pose3>(X(pNewNode->m_id)); 
          // const Rot3 dR = vro_p.rotation().between(cur_state.attitude()); 
          // const Vector3 dw = Rot3::Logmap(dR); 
          
          // Visual Odometry from camera frame to IMU frame 
          Pose3 inc_pose(pm->edge.transform.matrix());
          Pose3 Tu2c = *(gt_graph.mp_u2c); 
          inc_pose = (Tu2c)*inc_pose*(Tu2c).inverse();
          // Eigen::Matrix<double, 6, 6> Adj_Tuc = Tu2c.AdjointMap(); 
          // Eigen::Matrix<double, 6, 6> Inf_Tij = Adj_Tuc * pm->edge.informationMatrix * Adj_Tuc.transpose(); 
          // Matrix3 Inf_Rij = Inf_Tij.block<3,3>(0,0); 
          const Rot3 dR_vro = inc_pose.rotation(); 

          // cout <<"Inf_Rij: "<<endl<<Inf_Rij<<endl;
          // if(!MatrixCheck(Inf_Rij))
          {
            //ROS_INFO("Inf_Rij is rank deficient!"); 
          }

          // IMU Preintegration's measurement, Delta_Rij
          gtsam::PreintegratedCombinedMeasurements* pIMU = dynamic_cast<gtsam::PreintegratedCombinedMeasurements*>(imu->mp_combined_pre_imu); 
          Pose3 pre_p = gt_graph.mp_node_values->at<Pose3>(X(pNewNode->m_id-1));
          Pose3 inc_p = pre_p.transform_pose_to(cur_state.pose()); 
          const Rot3 dR_imu = inc_p.rotation(); 

          Matrix3 D_dRw_D_dR_imu; // D_dRw_D_dR_vro; 
          
          Rot3 dRw = dR_imu.between(dR_vro, D_dRw_D_dR_imu); // , D_dRw_D_dR_vro); 
          Matrix3 D_v_D_R; 
          const Vector3 dw = Rot3::Logmap(dRw, D_v_D_R); 
          Matrix3 D_dw_D_dR_imu = D_v_D_R * D_dRw_D_dR_imu; 
          // Matrix3 D_dw_D_dR_vro = D_v_D_R * D_dRw_D_dR_vro; 
          // const Vector3 dw = dR_imu.localCoordinates(dR_vro, D_dw_D_dR_imu, D_dw_D_dR_vro); 

          // const Matrix3 Sigma_R = pIMU->preintMeasCov().block<3,3>(6,6); 
          // noiseModel::Gaussian::shared_ptr IMU_noise = noiseModel::Gaussian::Covariance(pIMU->preintMeasCov());
          // Eigen::MatrixXd R = IMU_noise->thisR(); // this is private version 
          Eigen::MatrixXd Cov = pIMU->preintMeasCov(); 
          Eigen::MatrixXd Information = Cov.inverse(); 
          Matrix3 Inf_Rimu = Information.block<3,3>(0,0); 
          
          // Eigen::MatrixXd T = Cov * Information; 
          // cout <<"T: "<<endl<<T<<endl;
          // if(!MatrixCheck(Information))
          {
            // ROS_INFO("Information is rank deficient!"); 
          }
          // cout <<"Inf_Rimu: "<<endl<<Inf_Rimu<<endl;
          // cout <<"D_dw_D_dRimu"<<endl<<D_dw_D_dR_imu<<endl; 
          // cout <<"D_dw_D_dRvro"<<endl<<D_dw_D_dR_vro<<endl; 
            
          // Matrix3 Info1 = D_dw_D_dR_imu * Inf_Rimu * D_dw_D_dR_imu.transpose(); 
          // Matrix3 Info2 = D_dw_D_dR_vro * Inf_Rij * D_dw_D_dR_vro.transpose();
          // cout <<"Info1: "<<endl<<Info1<<endl;
          // cout <<"Info2: "<<endl<<Info2<<endl;
          Matrix3 Info_dw = D_dw_D_dR_imu * Inf_Rimu * D_dw_D_dR_imu.transpose();  // + D_dw_D_dR_vro * Inf_Rij * D_dw_D_dR_vro.transpose(); 
          double mahalanobis_dis = dw.dot(Info_dw*dw); 
          
          // cout <<"Info_dw: "<<endl<<Info_dw<<endl;
          // cout <<"dw: "<<endl<<dw<<endl;
          // cout <<"m_dis : "<<mahalanobis_dis<<endl; 
          // break; 

          // static ofstream ouf("reject_vro_list.log"); 
          // ouf << pm->edge.id2<<" "<<pm->edge.id1<<" "<<mahalanobis_dis<<" "<<dw(0)<<" "<<dw(1)<<" "<<dw(2)<<endl; 

          if(mahalanobis_dis > 40000)  // REJECT IT CI
          {
            static ofstream ouf("reject_vro_list.log"); 
            node_removed_by_chi2 = true; 
            // delete the factor 
            gt_graph.mp_fac_graph->remove(gt_graph.mp_fac_graph->size() - 1); 
            gt_graph.mp_new_fac->remove(gt_graph.mp_new_fac->size() - 1); 
            
            // record rejected edges 
            ouf << pm->edge.id2<<" "<<pm->edge.id1<<" "<<mahalanobis_dis<<" "<<dw(0)<<" "<<dw(1)<<" "<<dw(2)<<endl; 

            // update the node 
            gt_graph.mp_node_values->update(X(pNewNode->m_id), cur_state.pose()); 

            node_removed_by_chi2 = true; 
            ROS_WARN("test_vro_imu_graph.cpp: reject vro mahalanobis_dis: %f CI: %f dw: %f %f %f", mahalanobis_dis, CI, dw(0), dw(1), dw(2));
          }else{
          
            node_removed_by_chi2 = false; 
            // ROS_WARN("test_vro_imu_graph.cpp: vro mahalanobis_dis: %f CI: %f dw: %f %f %f", mahalanobis_dis, CI, dw(0), dw(1), dw(2));
          }

          
        }else{
            node_removed_by_chi2 = false; 
        }



 * */
