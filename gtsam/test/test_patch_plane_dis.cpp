
/*
 * Feb. 11, 2017 David Z 
 * 
 * Patch test for plane dis between two frames of SwissRanger 4000 
 *
 * */

#include <ros/ros.h>
#include <string> 
#include <tf/tf.h>
#include <iostream>
#include <map>
#include <pcl/common/transforms.h>
#include "sparse_feature_vo.h"
#include "SR_reader_cv.h"
#include "camera_node_ba.h"
#include "glob_def.h"
#include "pc_from_image.h"
#include "vtk_viewer.h"
#include "gtsam_graph.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "plane.h"
#include "plane_node.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/OrientedPlane3.h>

using namespace cv; 
using namespace std; 
using namespace gtsam; 

#define R2D(r) (((r)*180.)/M_PI)

string g_file_dir; 

void test_patch(int argc, char* argv[]);
void init_parameters(); 
// void testTwoFrameMatch(); 
void testTwoFrameMatch(int idi, int idj, CGraphGT& gt_graph, double & err, double & err_raw);
void print_tf(ostream& out, tf::Transform tT); 
double computePlaneDis(CPlane* pi, CPlane* pj, Pose3& Tij, Matrix6& Sij, double& raw_e);  // compute the M distance between (pj, Tij.transform(pi))

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_patch_plane_dis"); 
  ros::NodeHandle n;
  init_parameters(); 
  // testTwoFrameMatch(); 
  test_patch(argc, argv); 
  return 0; 
}

void test_patch(int argc, char* argv[])
{
  int idi = 2686;
  int idj = 2688; 
  string g_vro_results_file;
  string g_patch_log; 
  string g_output_log; 
  ros::NodeHandle nh("~"); 
  
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f2/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/eit_f23_r1/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/eit_f1_r1/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/eit_f5_r1/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f4_r1/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f5_r1/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/etas_f2_r1/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/eit_f23/sr4k")); // default parameters
  nh.param("sr_data_file_dir", g_file_dir, string("/home/david/work/data/sr4k/imu_bdat/eit_f5_r2/sr4k")); // default parameters

  // nh.param("vro_results_file", g_vro_results_file, string("vro_results.log_hybrid_add")); 
  // nh.param("vro_results_file", g_vro_results_file, string("vo_results_merge.log")); 
  nh.param("vro_results_file", g_vro_results_file, string("vro_results.log_hybrid")); 
  nh.param("vro_results_file", g_vro_results_file, string("vro_results.log_ba_5")); 

  // nh.param("patch_log", g_patch_log, string("num_vs_err.log")); 
  nh.param("patch_log", g_patch_log, string("input_match.log")); 
  nh.param("output_log", g_output_log, string("num_vs_err_new.log_vro"));

  if(argc > 1) 
    g_vro_results_file = argv[1]; 

   // set graph strcuture 
  CGraphGT gt_graph; 
   // VRO result 
  gt_graph.readVRORecord(g_vro_results_file); 
  if(gt_graph.mv_vro_res.size() <= 0)
  {
    ROS_ERROR("failed to load vro file : %s", g_vro_results_file.c_str()); 
    return ; 
  }

  ifstream inf(g_patch_log.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("failed to open file %s", g_patch_log.c_str());   
    return ; 
  }

  double e, er, ne, ner; 
  int n; 
  ofstream ouf(g_output_log.c_str()); 
  while(!inf.eof())
  {
    inf >> idj >> idi >>n ; // >> e >> er; 
    // ROS_INFO("handle record idj %d idi %d e: %lf er :%lf", idj, idi, e, er); 
    // if(idj != 401 || idi != 399) continue; 
    if(inf.eof()) break; 
    if(n < 20) // 20, 25, 30
    {
      // if(e >= 0)
        testTwoFrameMatch(idi, idj, gt_graph, ne, ner); 
    }else
    {
      // e = er = ne = ner = 0;
      ne = ner = 0; 
    }
    ouf << idj <<"\t"<< idi<<"\t"<<n<<"\t" ; // << e <<"\t"<<er<<"\t"; 
    ouf <<ne<<"\t"<<ner<<endl; 
  }
  inf.close(); 
  ouf.close(); 
  return ; 
}


void testTwoFrameMatch(int idi, int idj, CGraphGT& gt_graph, double & err, double & err_raw)
{
  // VRO 
  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  sr4k.setDepthScale(0.001); 
  CCameraNode::set_cam_cov(sr4k); 
  CCameraNode::gb_dis_match_point = true; // whether to see the result of 

  for(int k=0; k<gt_graph.mv_vro_res.size() && ros::ok(); k++)
  {
    MatchingResult* pm = gt_graph.mv_vro_res[k]; 
    if(pm->edge.id2 != idj || pm->edge.id1 != idi) continue; 
  
    Matrix4 TMij = pm->final_trafo.cast<double>(); 
    Matrix4 TMji = TMij.inverse(); 
    Pose3 Tij (TMij); 
    Matrix6 S_Tij = pm->edge.informationMatrix.inverse(); 

    // getframe 
    stringstream sni, snj; 
    sni << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<idi<<".bdat"; 
    snj << g_file_dir<<"/d1_"<<setfill('0')<<setw(7)<<idj<<".bdat"; 

    // CSReader r4k; 
    CSReadCV r4k; 

    // generate imgs and dpts 
    cv::Mat tar_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
    cv::Mat src_cv_d_img(SR_HEIGHT, SR_WIDTH, CV_16UC1); 
    cv::Mat src_cv_i_img; 
    cv::Mat tar_cv_i_img; 

    if(!r4k.readOneFrameCV(snj.str().c_str(), src_cv_i_img, src_cv_d_img))
    {
      ROS_INFO("%s failed to read file %s", __FILE__, snj.str().c_str()); 
      return ;
    }
    if(!r4k.readOneFrameCV(sni.str().c_str(), tar_cv_i_img, tar_cv_d_img))
    {
      ROS_INFO("%s failed to read file %s", __FILE__, sni.str().c_str()); 
      return ;
    }

    // generate node 
    CSparseFeatureVO vo(sr4k); 
    CCameraNodeBA* nni = new CCameraNodeBA(); 
    CCameraNodeBA* nnj = new CCameraNodeBA(); 
    vo.featureExtraction(tar_cv_i_img, tar_cv_d_img, 0.001, *nni); 
    vo.featureExtraction(src_cv_i_img, src_cv_d_img, 0.001, *nnj); 

    // generate plane node 
    CPlaneNode* pni = new CPlaneNode(); 
    CPlaneNode* pnj = new CPlaneNode();
    int ret = pni->extractPlanes(tar_cv_i_img, tar_cv_d_img, &sr4k); 
    if(ret <= 0)
    {
      ROS_INFO("what? pni has no plane ,idi = %d", idi); 
      return ;
    }else{
      ROS_INFO("pni has %d planes ", pni->mv_planes.size()); 
    }
    
    ret = pnj->extractPlanes(src_cv_i_img, src_cv_d_img, &sr4k); 
    if(ret <=0)
    {
      ROS_INFO("what? pnj has no plane, idj = %d,", idj ); 
      return; 
    }else{
      ROS_INFO("pnj has %d planes ", pnj->mv_planes.size()); 
    }

    double COS10 = cos(10.*M_PI/180.); 
    double COS5 = cos(5.*M_PI/180.);
    // double
    err = 0; err_raw = 0; 
    int best_i = -1, best_j = -1; 
    int i, j;
    Vector4 bnej; 

    Vector4 ni;
    Vector4 nj;
    Vector4 nej; 

    // find matches 
    for(i=0; i<pni->mv_planes.size(); i++)
    {
      CPlane* pi = pni->mv_planes[i]; 
      CPlane* pj = NULL; 
      ni << pi->nx_, pi->ny_, pi->nz_, pi->d1_; 
      OrientedPlane3 Pi(ni); 
      OrientedPlane3 PEj = Pi.transform(Tij); 
      nej = PEj.planeCoefficients(); 
      cout<<"i = "<<i<<" pi: "<<endl<<ni<<endl;
      bool found_it = false; 
      for(j=0; j<pnj->mv_planes.size(); j++)
      {
        pj = pnj->mv_planes[j]; 
        nj << pj->nx_, pj->ny_, pj->nz_, pj->d1_; 
        cout <<"j = "<<j <<" nj: "<<endl<<nj<<endl; 
        // double COSA = nej.transpose()*nj - nej(3)*nj(3); 
        double COSA = ni.transpose()*nj - ni(3)*nj(3); 
        cout <<"COSA = "<<COSA<<" angle = "<<acos(COSA)*180./M_PI<<endl;
        if(fabs(COSA) >= COS10 && fabs(ni(3)- nj(3)) <= 0.2)
        {
          // got it  
          found_it = true; 
          bnej = nej; 
          break; 
        }
      }
      if(found_it)
      {
        double COSA = nej.transpose()*nj - nej(3)*nj(3); 
        double distance = fabs(nej(3) - nj(3)); 
        
        double p_dis, p_raw_dis; 
        double angle = acos(COSA)*180./M_PI;

        if(angle >= 5 || distance >= 0.1) 
        {
          p_dis = 11; p_raw_dis = 11; 
        }else{
          p_dis = computePlaneDis(pi, pj, Tij, S_Tij, p_raw_dis); 
        }
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
/*
    // display result 
    ROS_INFO("pni has %d planes, pnj has %d planes, best match from pi %d to pj %d whitened err %lf, unwhitened err %lf ", pni->mv_planes.size(), pnj->mv_planes.size(), best_i, best_j, err, err_raw); 

    // display the best matched planes 
    if(best_i == -1 || best_j == -1) break; 
    
    CPlane* bPi = pni->mv_planes[best_i]; 
    CPlane* bPj = pnj->mv_planes[best_j]; 
    {
      Vector4 ni, nj; 
      ni << bPi->nx_, bPi->ny_, bPi->nz_, bPi->d1_; 
      nj << bPj->nx_, bPj->ny_, bPj->nz_, bPj->d1_; 

      cout <<"pi's parameters: "<<endl<<ni<<endl; 
      cout <<"pj's parameters: "<<endl<<nj<<endl;
      cout <<"pi in Pj pej's parameters: "<<endl<<bnej<<endl; 
    }

    // show them in point cloud 
    CloudPtr pci(new Cloud); 
    CloudPtr pcj(new Cloud); 
    CloudPtr pcj_ni(new Cloud); 
    generatePointCloud(tar_cv_i_img, tar_cv_d_img, 0.001, sr4k, *pci);
    generatePointCloud(src_cv_i_img, src_cv_d_img, 0.001, sr4k, *pcj);
    pcl::transformPointCloud(*pcj, *pcj_ni,  TMij.cast<float>()); // mr.final_trafo); 
  
    markColor(*pci, pni->mv_indices[best_i], GREEN); 
    markColor(*pcj_ni, pnj->mv_indices[best_j], RED); 
    *pci+= *pcj_ni;

    CVTKViewer<pcl::PointXYZRGBA> v;
    // v.getViewer()->addCoordinateSystem(0.2, 0, 0); 
    v.addPointCloud(pci, "pci + pcj"); 
    while(!v.stopped())
    {
      v.runOnce(); 
      usleep(100*1000); 
    }
*/
    break; 
  }

  ROS_INFO("OK, finish it"); 
 
  return ;

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
  // parameters configure 
  CParams* pP = CParams::Instance(); 
  pP->m_feature_detector_type = "SIFT";  // SIFTGPU 
  pP->m_feature_descriptor_type = "SIFT";  // SIFTGPU
  pP->m_feature_match_type = "FLANN"; 
  pP->m_nn_distance_ratio = 0.5; // 0.95 for SIFTGPU, 0.5-0.7 for SIFT 
  pP->m_max_dist_for_inliers = 0.05;  
  pP->m_max_num_features = 500; 
  pP->m_ransac_iterations = 5000;
  pP->m_min_matches = 4; 
}
