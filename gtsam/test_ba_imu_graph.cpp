/*
 * test gtsam graph slam in offline mode
 * 
 * using ba and imu measurement 
 *
 * */

#include <ros/ros.h> 
#include <sstream>
#include <string>
#include "SR_reader_cv.h"
#include "sparse_feature_vo.h"
#include "camera_node.h"
#include "camera_node_ba.h"
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
using symbol_shorthand::Q; // Visual features (x, y, z)

void init_parameters(); 
void test_with_sr4k(); 
void test_read_vro();

void copyMat(cv::Mat& f, cv::Mat& t);

string g_file_dir; 
string g_file_pre; 
string g_file_suf;
string g_data_name;

bool g_chi2_test = false;   // whether to use Chi2 test to delete VRO edges 
bool g_plane_factor = false; // whether to add plane factors into graph 
bool g_view_plane = false;  // whether to view the result of plane propagation 
bool g_use_imu = false; 
int g_f_start; 
int g_f_end;

string g_imu_file; // imu measurement file 
string g_imu_time_file; // sr timestamp file, which is used to synchronize between camera data and imu data 
string g_vro_results_file; // where the vro results are stored 

bool loadImgTime(map<int, double>&); // load synchronized time stamps

int main(int argc, char* argv[])
{
  // CImuBase::getParam(); 
  ros::init(argc, argv, "test_ba_imu_graph"); 
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
    ROS_ERROR("failed to read time file %s", g_imu_time_file.c_str());
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

  // CCameraNode* pNewNode = new CCameraNode();
  CCameraNode* pNewNode = new CCameraNodeBA(); 

  spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
  pNewNode->m_seq_id = g_f_start;
  gt_graph.firstNode(pNewNode, false);
  imu->setStartPoint(img_times[g_f_start]);  // first measurement 

  if(g_plane_factor) // use plane factor 
  {
    CPlaneNode* p1 = new CPlaneNode(); 
    int ret = p1->extractPlanes(i_img, d_img, &sr4k); 
    if(ret == 0)
    {
       delete p1; p1 = NULL;
    }
    gt_graph.firstPlaneNode(p1); 
  }

  // traversely visit the vro matching results 
  int cur_frame_id = g_f_start; 
  int cur_imu_id = cur_frame_id; 
  int cur_node_id = cur_imu_id; 
  
  ROS_INFO("at line %d start to read vro results", __LINE__);

  // record consequent NavState 
  NavState pre_state, cur_state; 
  cv::Mat pre_rgb, pre_dpt; 

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
        // CCameraNode* pNewNode = new CCameraNode();
        CCameraNode* pNewNode = new CCameraNodeBA(); 
        pNewNode->m_id = gt_graph.m_graph_map.size();  
        pNewNode->m_seq_id = pm->edge.id2; 
        // gt_graph.m_graph_map[pNewNode->m_id] = pNewNode; 
        
        Pose3 Pj = gt_graph.mp_node_values->at<Pose3>(X(pNewNode->m_id-1)); 
        NavState cur_p; 
        PreintegratedCombinedMeasurements* preint_imu_combined;
        
        // Pose change 
        Pose3 pre_p = gt_graph.mp_node_values->at<Pose3>(X(pNewNode->m_id-1));
        Pose3 inc_p = pre_p.transform_pose_to(Pj); 
        
        // extract features 
        spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
        // bool succeed = gt_graph.bundleAdjust(pm, pNewNode, &sr4k); 
        // bool succeed = gt_graph.vroAdjust(pm, pNewNode, &sr4k); 
        bool succeed = true; 

        /*
         CCameraNode* preNode = gt_graph.m_graph_map[pNewNode->m_id-1]; 
      
        // find matches between two frames 
        CCameraNodeBA* nj = dynamic_cast<CCameraNodeBA*>(pNewNode); 
        CCameraNodeBA* ni = dynamic_cast<CCameraNodeBA*>(preNode); 

        Eigen::Matrix4d T = inc_p.matrix(); 
        Eigen::Matrix4f Tij  = T.cast<float>(); 

        if(pm->edge.informationMatrix(0,0) != 10000) // a valid transformation 
        {
          Tij = pm->final_trafo; 
        }

        Eigen::Matrix4f Tji = Tij.inverse(); 
        map<int, int> matches = nj->matchNodePairBA(ni, Tji, &sr4k);
      
        if(!g_use_imu) // add pose 
        {
          gt_graph.mp_node_values->insert(X(pNewNode->m_id), Pj); 
          gt_graph.mp_new_node->insert(X(pNewNode->m_id),  Pj); 
        }

        gt_graph.addToGTSAM(ni, nj, matches, &sr4k); 
        */
        bool valid_match = gt_graph.addNodeOffline(pNewNode, pm, !g_use_imu);
        // if(!valid_match) // failed to add vo factor, so the pose node has not been added,        
        {
          // manually add node into graph 
          // gt_graph.m_graph_map[pNewNode->m_id] = pNewNode; 
        }
        
        if(g_use_imu)
        {
          // imu preintegration,  add imu factor between these two nodes 
          cur_p = imu->predictNext(img_times[cur_imu_id]);

          // current pose 
          Pj = cur_p.pose(); 

          // add imu measurement into graph 
          cur_node_id = pNewNode->m_id; 
          // PreintegratedCombinedMeasurements* 
          preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu->mp_combined_pre_imu); 
          CombinedImuFactor imu_factor(X(cur_node_id-1), V(cur_node_id-1), 
              X(cur_node_id),    V(cur_node_id), 
              B(cur_node_id-1), B(cur_node_id), *(preint_imu_combined));

          gt_graph.mp_fac_graph->add(imu_factor); 
          gt_graph.mp_new_fac->add(imu_factor); 
          // ROS_INFO("what>? add IMU factor and Nodes "); 
          gt_graph.addToGTSAM(cur_p, cur_node_id, !valid_match); 
        }




        // whether add plane node  
        if(g_plane_factor)
        {
          int pre_plane_num = gt_graph.m_plane_landmark_id; 
          // if(!valid_match)
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
            /*
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
            }*/

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

          } // else{
            // TODO: 
            // gt_graph.mv_plane_nodes[pNewNode->m_id] = NULL; 
          // } 
          
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
        
        int pre_id1 = pm->edge.id1; 
        int pre_id2 = pm->edge.id2; 
        gt_graph.correctMatchingID(pm); 
        CCameraNode* pNode = gt_graph.m_graph_map[pm->edge.id2]; 

        pm->edge.id1 = pre_id1; 
        pm->edge.id2 = pre_id2;

        // bool succeed = gt_graph.bundleAdjust(pm, pNode, &sr4k); 
        // bool succeed = gt_graph.vroAdjust(pm, pNewNode, &sr4k); 
        bool succeed = true;
        if(succeed)
          gt_graph.addEdgeOffline(pm); 

/*
        int pre_id1 = pm->edge.id1; 
        int pre_id2 = pm->edge.id2; 
        gt_graph.correctMatchingID(pm); 
          
        // gt_graph.addEdgeOffline(pm); 
        // CCameraNodeBA* ni = dynamic_cast<CCameraNodeBA*>(gt_graph.m_graph_map[pm->edge.id1]); 
        // CCameraNodeBA* nj = dynamic_cast<CCameraNodeBA*>(gt_graph.m_graph_map[pm->edge.id2]); 
        
        pm->edge.id1 = pre_id1; 
        pm->edge.id2 = pre_id2;

        Eigen::Matrix4f Tji = pm->final_trafo.inverse(); 
        map<int, int> matches = nj->matchNodePairBA(ni, Tji, &sr4k); 
        
        if(matches.size() > 0)
        {
          gt_graph.addToGTSAM(ni, nj, matches, &sr4k);
        }
*/
        j++; 
      }
      
      i = j - 1; // update i
      
      // gt_graph.mp_new_fac->print("loop fac: ");
    }

    // optimize graph 
    gt_graph.optimizeGraphIncremental(); 
    // gt_graph.optimizeGraphBatch();

    // reset imu bias parameters
    if(g_use_imu)
    {
      imu->resetPreintegrationAndBias(gt_graph.mp_node_values->at<imuBias::ConstantBias>(B(cur_node_id))); 
      pre_state = NavState(gt_graph.mp_node_values->at<Pose3>(X(cur_node_id)), gt_graph.mp_node_values->at<Vector3>(V(cur_node_id))); 
      imu->setState(pre_state); 
    }
    // gt_graph.mp_fac_graph->print("current graph: "); 

    // print graph to see what's going on 
    // gt_graph.mp_node_values->print("current estimate : ");
    // if(i>=3) break; 
  }
 
  string mode = "_ba"; 
  if(g_use_imu) mode += "_vio";
  if(g_plane_factor) mode +="_pl"; 
  // gt_graph.writeG2O("before_opt.g2o");
  ROS_INFO("before optimization error is %lf", gt_graph.error());
  string dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + mode + "_before.ply";
  gt_graph.trajectoryPLY(dataname, CG::BLUE); 
  gt_graph.optimizeGraphBatch(); 

  ROS_INFO("after optimization error is %lf", gt_graph.error()); 
  dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + mode + "_after.ply"; 
  gt_graph.trajectoryPLY(dataname, CG::RED); 

  // record trajectory for later mapping 
  dataname = CGTParams::Instance()->m_output_dir + "/" + g_data_name + mode + "_after_trajectory.log"; 
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
  ifstream inf(g_imu_time_file);
  if(!inf.is_open())
  {
    ROS_ERROR("%d failed to load camera timestamp %s", __LINE__, g_imu_time_file.c_str()); 
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
  np.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f2")); // default parameters 
  np.param("sr_data_prefix", g_file_pre, string("d1")); 
  np.param("sr_data_suffix", g_file_suf, string("bdat")); 
  np.param("sr_start_frame", g_f_start, 1); 
  np.param("sr_end_frame", g_f_end, 500); 
  np.param("sr_data_name", g_data_name, string("etas_f2"));

  np.param("imu_file", g_imu_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/imu_v100.log"));
  np.param("imu_time_file", g_imu_time_file, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/sr4k/timestamp.log")); 
  np.param("vro_results_file", g_vro_results_file, string("/home/david/.ros/vro_results/etas_f2_vro_results.log")); 
  np.param("chi2_for_vro", g_chi2_test, g_chi2_test); 
  np.param("plane_aided", g_plane_factor, true);
  np.param("view_plane", g_view_plane, g_view_plane); 
  np.param("use_imu", g_use_imu, g_use_imu); 

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

}

