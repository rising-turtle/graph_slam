/*
 *  May. 11 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  record a video of reconstructing the 3D map, given the trajectory and point clouds 
 *
 *
 * */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "pc_from_image.h"
#include "SR_reader_cv.h"
#include "vtk_viewer.h"

using namespace std; 
using namespace gtsam;

struct trajItem
{
  int id; int sid; // sequence id 
  float px, py, pz; // euler angle roll, pitch, yaw; 
  float qx, qy, qz, qw; // quaternion
};

void setTu2c(Pose3& Tu2c);
bool showPCDIncremental(string traj_f, string img_dir, CamModel&); 
bool readTraj(std::string f, vector<struct trajItem>& );

template<typename PointT>
void filterPointCloud(float _voxel_size, typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out)
{
  // voxelgrid filter
  pcl::VoxelGrid<PointT> vog;
  vog.setInputCloud(in); 
  vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
  vog.filter(*out);
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "map_video"); 
  ros::NodeHandle n; 
  // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); \

  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;

  string traj_f("/home/david/.ros/vio/eit_f5_r2/eit_f5_r2_pvio_trajectory.log"); 
  string img_dir("/home/david/work/data/sr4k/imu_bdat/eit_f5_r2/sr4k"); 

  cout<<"usage: ./map_video img_dir trajectory_file"<<endl; 

  if(argc >= 2)
    img_dir = argv[1]; 
  if(argc >= 3)
    traj_f = argv[2]; 
  
  showPCDIncremental(traj_f, img_dir, sr4k); 

  return 1; 
}


bool showPCDIncremental(string traj_f, string img_dir, CamModel& cam)
{
  vector<trajItem> vt; 
  if(!readTraj(traj_f, vt))
  {
    return false; 
  }
  
  CloudPtr pc_w(new Cloud); 
  cv::Mat i_img, d_img;   
  CSReadCV r4k; 
  
  Pose3 Pu2c; 
  setTu2c(Pu2c); 

  CVTKViewer<pcl::PointXYZRGBA> v;
  v.getViewer()->addCoordinateSystem(1.0, "cloud", 0); 
  v.getViewer()->setBackgroundColor(0.05, 0.05, 0.05, 0); 
  v.addPointCloud(pc_w, "pwi"); 
  v.addPointCloud(pc_w, "pw"); 
  for(int i=0; i<vt.size() && i < 1000; i++)
  {
    trajItem& ti = vt[i]; 
    
    // This is a bug in GTSAM, where the correct sequence is R(w, x, y, z) not R(x, y, z, w)
    Rot3 R(ti.qw, ti.qx, ti.qy, ti.qz); 
    // Rot3 R(ti.qx, ti.qy, ti.qz, ti.qw); 

    Point3 t; t << ti.px, ti.py, ti.pz; 
    Pose3 Pw2j = Pose3(R, t); 
    Pose3 Pw2c = Pw2j * Pu2c; 

    // Pose3 Pj2w = Pw2j.inverse(); 
    // Point3 view_pw(0, 0, 1);  // camera view point 
    // Point3 up_pw(0, -1, 0);   // camera up direction 
    // Point3 view_pj = Pj2w * view_pw; 
    // Point3 up_pj = Pj2w * up_pw; 

    Point3 pos_pc(0, -2, -5);  // camera pose
    Point3 view_pc(0, 0, 5);  // camera view point 
    Point3 up_pc(0, -1, 0);   // camera up direction 
    Point3 view_pw = Pw2c * view_pc; 
    Point3 up_pw = Pw2c * up_pc; 
    Point3 pos_pw = Pw2c * pos_pc; 

    // get image 
    stringstream ss; 
    ss<<img_dir<<"/d1_"<<setfill('0')<<setw(7)<<ti.sid<<".bdat";  // setw(4)
    r4k.readOneFrameCV(ss.str(), i_img, d_img);

    // show images 
    cv::imshow("intensity img", i_img); 
    cv::imshow("depth img", d_img); 
    cv::waitKey(3); 

    // point cloud 
    CloudPtr pci(new Cloud);
    CloudPtr pwi(new Cloud);
    generatePointCloud(i_img, d_img, 0.001, cam, *pci); 
    
    Eigen::Matrix4d T = Pw2c.matrix();  
    // pcl::transformPointCloud(*pci, *pwi, Tw2j.cast<float>()); // transform into global 
    pcl::transformPointCloud(*pci, *pwi, T.cast<float>()); // transform into global 
    
    // add to global point cloud
    *pc_w += *pwi; 

    // voxel grid filter
    CloudPtr tmp(new Cloud); 
    filterPointCloud<pcl::PointXYZRGBA>(0.05, pc_w, tmp); 
    pc_w.swap(tmp); 

    // set pose of the current point cloud 
     Eigen::Matrix<float, 4,4> Tf = T.cast<float>(); 
     Eigen::Affine3f Tf3(Tf); 
     pwi->sensor_origin_ = Tf.block<4,1>(0,3); 
     pwi->sensor_orientation_ = Eigen::Quaternionf(ti.qw, ti.qx, ti.qy, ti.qz); 

    if(!v.stopped())
    {
      // show the point cloud 
      v.getViewer()->updatePointCloud(pwi, "pwi"); 
      v.getViewer()->updatePointCloud(pc_w, "pw");
      // v.getViewer()->resetCameraViewpoint("pwi"); 
      // v.getViewer()->setCameraPosition(ti.px, ti.py, ti.pz, view_pj.x(), view_pj.y(), view_pj.z(), up_pj.x(), up_pj.y(), up_pj.z());
      v.getViewer()->setCameraPosition(pos_pw.x(), pos_pw.y(), pos_pw.z(), view_pw.x(), view_pw.y(), view_pw.z(), up_pw.x(), up_pw.y(), up_pw.z());
      v.getViewer()->removeCoordinateSystem(); 
      v.getViewer()->addCoordinateSystem(0.5, Tf3);
      v.runOnce(10); 
      usleep(10*1000); 
    }
  }
  
return true; 
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
  while(inf.getline(buf, 4096))
  {
    trajItem ti; 
    // sscanf(buf, "%d %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.roll, &ti.pitch, &ti.yaw, &ti.sid); 
    sscanf(buf, "%d %f %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw, &ti.sid); 
    t.push_back(ti); 
  }

  cout << " read "<<t.size()<<" trajectory items"<<endl;
  return true;

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




