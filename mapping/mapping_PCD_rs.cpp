/*
 * Apr. 8, 2017, He Zhang, hxzhang1@ualr.edu
 *
 * Generate 3D map: 
 *     input:  a trajectory.log and img dir where the images are stored 
 *     output: a .pcd file, shown the 3d point cloud 
 *
 * */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
// #include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "pc_from_image.h"
#include "sparse_feature_vo.h"
#include "cam_model.h"
#include "SR_reader_cv.h"
#include "vtk_viewer.h"

using namespace gtsam; 
using namespace std; 

#define D2R(d) (((d)*M_PI)/180)

// typedef pcl::PointXYZRGB  Point;
typedef pcl::PointCloud<pcl::PointXYZRGB>  CloudL;  
typedef typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudLPtr;

string fname("/home/david/work/data/sr4k/imu_bdat/etas_f5/imu_v100.log"); 

double voxel_grid_size = 0.02; 
double z_far = 1.2; // 0 - z_far is kept 

struct trajItem
{
  int id; int sid; // sequence id 
  // double timestamp; 
  // string timestamp; 
  float px, py, pz; // euler angle roll, pitch, yaw; 
  float qx, qy, qz, qw; // quaternion
};

void setTu2c(Pose3& Tu2c); 
void headerPLY(std::ofstream& ouf, int vertex_number);
bool mapPCD(std::string f, std::string img_dir, std::string outPCD, int skip, int t_skip, float depth_scale, CSparseFeatureVO* pSF, CamModel&,  int* rect = 0);
bool readTraj(std::string f, vector<struct trajItem>& );
void LoadImages(string fAsso, vector<string> & vRGB, vector<string>& vDPT, vector<double>& vTime); 
int  findIndex(double t, vector<double>& vTime); 

template<typename PointT>
void filterPointCloud(float _voxel_size, typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out)
{
  // voxelgrid filter
  pcl::VoxelGrid<PointT> vog;
  vog.setInputCloud(in); 
  vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
  vog.filter(*out);
}

template<typename PointT>
void passThroughZ(float z_far, typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out)
{
    pcl::PassThrough<PointT> pf; 
    pf.setInputCloud(in); 
    pf.setFilterFieldName("z"); 
    pf.setFilterLimits(0.0, z_far); 
    pf.filter(*out); 
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mapping_PCD_rs"); 
  ros::NodeHandle n; 
  // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); \

  // CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  // CamModel rs_F200(621.94176, 625.3561, 318.77554, 236.21507, 0.08011, -0.56891); 
  CamModel rs435(617.306, 617.714, 326.245, 239.974); 
  CamModel cam = rs435; 
  cam.m_rows = 480; // height
  cam.m_cols = 640; // width
  // sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;
  CSparseFeatureVO spr_vo(cam);

  // parameters 
  ros::NodeHandle np("~"); 
  int skip = 2; 
  int t_skip = 1; 
  string trajFile;  // the trajectory log file 
  string imgDir;    // images dir 
  string outPCD;    // the output PLY file 
  
  int su, sv, eu, ev ; 

  np.param("trajectory_file", trajFile, trajFile); 
  np.param("img_directory", imgDir, imgDir); 
  np.param("output_PCD_file", outPCD, outPCD); 
  np.param("downsample_skip", skip, skip); 
  np.param("trajectory_skip", t_skip, t_skip); 
  np.param("top_left_u", su, 0); 
  np.param("top_left_v", sv, 0); 
  np.param("bot_right_u", eu, cam.m_cols); 
  np.param("bot_right_v", ev, cam.m_rows); 
  np.param("voxel_grid_size", voxel_grid_size, voxel_grid_size); 
  np.param("z_pass_through", z_far, z_far); 

  int rect[4] = {su, sv, eu, ev}; 

  cout <<"rect "<<rect[0]<<" "<<rect[1]<<" "<<rect[2]<<" "<<rect[3]<<endl;

  mapPCD(trajFile, imgDir, outPCD, skip, t_skip, depth_scale, &spr_vo, cam, &rect[0]);

  ROS_INFO("Finish mapping into %s!", outPCD.c_str());

  return 0; 
}

bool mapPCD(std::string f, std::string img_dir, std::string outPCD, int skip, int t_skip, float depth_scale, CSparseFeatureVO* pSF, CamModel& cam, int* rect)
{
  // output file 
  // ofstream ouf(outPLY.c_str()); 
  // if(!ouf.is_open())
  {
   // cout <<" failed to open output PLY file : "<<outPLY<<endl; 
   // return false; 
  }

  // generate a global point cloud 
  vector<trajItem> t; 
  if(!readTraj(f, t))
  {
    return false; 
  }
  
  ROS_INFO("succeed to load %d pose ", t.size()); 

  vector<float> pts_loc; 
  vector<unsigned char> pts_col;
  cv::Mat i_img, d_img; 
  // CSReadCV r4k; 

  ros::NodeHandle np("~"); 
  string record_file(""); 
  np.param("record_file", record_file, record_file); 
  
  vector<string> vRGBs; vector<string> vDPTs; vector<double> vTimes; 
  LoadImages(record_file, vRGBs, vDPTs, vTimes); 

  // transform point from camera to imu 
  // Pose3 Tu2c ; 
  // setTu2c(Tu2c); 
  
  CloudLPtr pc_w(new CloudL); 

  for(int i=0; i<t.size(); i+= t_skip)
  {
    trajItem& ti = t[i]; 

    // construct pose3
    // Rot3 R = Rot3::RzRyRx(ti.roll, ti.pitch, ti.yaw); 
    
    // This is a bug in GTSAM, where the correct sequence is R(w, x, y, z) not R(x, y, z, w)
    Rot3 R(ti.qw, ti.qx, ti.qy, ti.qz); 
    // Rot3 R(ti.qx, ti.qy, ti.qz, ti.qw); 
    Point3 t; t << ti.px, ti.py, ti.pz; 
    Pose3 p = Pose3(R, t); 

    // get image 
    stringstream ss_rgb, ss_dpt; 
    // ss<<img_dir<<"/d1_"<<setfill('0')<<setw(7)<<ti.sid<<".bdat";  // setw(4)
    // r4k.readOneFrameCV(ss.str(), i_img, d_img);
  
    /* 
    int index = findIndex(ti.timestamp, vTimes); 
    if(index < 0)
    {
      cout <<"mapping_PLY_rs.cpp: find to find timestamp: "<<ti.timestamp<<endl;
      break; 
    }*/

    // read image 
    // ss_rgb << img_dir <<"/color/"<<ti.timestamp<<".png"; 
    // ss_dpt << img_dir <<"/depth/"<<ti.timestamp<<".png"; 
    int index = ti.sid - 1;
    ss_rgb << img_dir <<"/"<<vRGBs[index]; 
    ss_dpt << img_dir <<"/"<<vDPTs[index]; 

    i_img = cv::imread(ss_rgb.str().c_str(), -1); 
    d_img = cv::imread(ss_dpt.str().c_str(), -1); 
    if(i_img.data == NULL || d_img.data == NULL)
    {
      ROS_ERROR("failed to load camera data at dir %s with timestamp = %lf", img_dir.c_str(), /*ti.timestamp*/ vTimes[ti.sid-1]); 
      ROS_WARN("ss_rgb = %s ss_dpt = %s", ss_rgb.str().c_str(), ss_dpt.str().c_str());
      break; 
    }
    
    CloudLPtr pci(new CloudL);
    CloudLPtr pwi(new CloudL); 
    generatePointCloud(i_img, d_img, 0.001, cam, *pci); 
    
    Eigen::Matrix4d T = p.matrix(); 
    
    {
	// pass through filter 
	CloudLPtr tmp(new CloudL); 
	// passThroughZ<pcl::PointXYZRGBA>(z_far, pci, tmp); 
	passThroughZ<pcl::PointXYZRGB>(z_far, pci, tmp); 
	pci.swap(tmp); 
    }

    pcl::transformPointCloud(*pci, *pwi, T.cast<float>()); // transform into global 

    *pc_w += *pwi; 
    {
	// voxel grid filter
	CloudLPtr tmp(new CloudL); 
	// filterPointCloud<pcl::PointXYZRGBA>(voxel_grid_size, pc_w, tmp); 
	filterPointCloud<pcl::PointXYZRGB>(voxel_grid_size, pc_w, tmp); 
	pc_w.swap(tmp); 
    }
    ROS_INFO("handle frame at timestamp %lf, add %d pts", /*ti.timestamp*/ vTimes[ti.sid-1], pwi->points.size());
  }
  
  // output into file 
  // save pci 
  pcl::io::savePCDFile(outPCD, *pc_w); 

  // see it
  // CVTKViewer<pcl::PointXYZRGBA> v;
  CVTKViewer<pcl::PointXYZRGB> v;

  v.getViewer()->addCoordinateSystem(1.0, 0, 0, 0); 
  v.addPointCloud(pc_w, "PC in world"); 
  while(!v.stopped())
  {
    v.runOnce(); 
    usleep(100*1000); 
  }

  // ouf.flush(); 
  // ouf.close(); 
return true;  
}
  
void headerPLY(std::ofstream& ouf, int vertex_number)
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


bool readTraj(std::string f, vector<struct trajItem>& t)
{

  ifstream inf(f.c_str()); 
  if(!inf.is_open())
  {
    cout<<"failed to open file : "<<f<<endl;
    return false;
  }

  char buf[4096]; 
  char b[21]={0}; 
  while(inf.getline(buf, 4096))
  {
    trajItem ti; 
    // sscanf(buf, "%d %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.roll, &ti.pitch, &ti.yaw, &ti.sid); 
    // sscanf(buf, "%d %f %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw, &ti.sid); 
    // sscanf(buf, "%lf %f %f %f %f %f %f %f", &ti.timestamp, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw); 
    sscanf(buf, "%d %f %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw, &ti.sid); 
    // ti.timestamp = string(b); 
    t.push_back(ti); 
    // if(t.size() < 10)
    {
      // printf("read %lf %f %f %f %f %f %f %f\n", ti.timestamp.c_str(), ti.px, ti.py, ti.pz, ti.qx, ti.qy, ti.qz, ti.qw);
    }
  }

  cout << " read "<<t.size()<<" trajectory items"<<endl;
  return true;

}

int findIndex(double t, vector<double>& vTime)
{
  for(int i=0; i<vTime.size(); i++)
  {
    if(vTime[i] == t)
      return i; 
    if(vTime[i] > t)
      return -1; 
  }
  return -1; 
}


void LoadImages(string fAsso, vector<string> & vRGB, vector<string>& vDPT, vector<double>& vTime)
{
  ifstream inf(fAsso.c_str()); 
  double t; 
  string sRGB, sDPT; 
  while(!inf.eof())
  {
    string s; 
    getline(inf, s); 
    if(!s.empty())
    {
      stringstream ss; 
      ss << s; 
      ss >> t;
      vTime.push_back(t); 
      ss >> sRGB; 
      vRGB.push_back(sRGB); 
      ss >> t; 
      ss >> sDPT; 
      vDPT.push_back(sDPT); 
    }
  }
  cout <<"mapping_PLY_rs.cpp: load "<<vTime.size()<<" records"<<endl;
}


void setTu2c(Pose3& Tu2c)
{
  //// Body/IMU Coordinate System 
  //          
  //             X
  //            /
  //           /
  //          /_____ Y
  //          |
  //          |
  //        Z | 
  //
  //        
  //   CAMERA Coordinate System    
  //           Z 
  //          /
  //         / 
  //        /  
  //        ------- X
  //        |
  //        |
  //        |Y
  //

  float p = 0; 
  Rot3 R_g2b = Rot3::RzRyRx( M_PI/2., 0. , M_PI/2.); // roll, pitch, yaw, in BODY coordinate system 
  Rot3 R_b2o = Rot3::RzRyRx(p ,0 , 0); 
  Rot3 R_g2o = R_g2b * R_b2o; 
  // Rot3 R_g2o = R_b2o * R_g2b; 

  Point3 t = Point3::Zero(); 
  // (*mp_u2c) = Pose3::Create(R_g2o, t);
  Tu2c = Pose3::Create(R_g2o, t); 
   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<Tu2c*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 

  return ; 
}
