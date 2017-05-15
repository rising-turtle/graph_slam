/*
 *  May. 12 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  record a video of reconstructing the 3D map, given multilpe trajectories and point clouds 
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
bool readTrajPly(std::string f, vector<struct trajItem>& t); 
void addLine( CVTKViewer<pcl::PointXYZRGBA>& , float sx, float sy, float sz, float ex, float ey, float ez, COLOR c, string id); 
void moveCameraInArc(CVTKViewer<pcl::PointXYZRGBA>& v); 

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
  ros::init(argc, argv, "map_video_mtraj"); 
  ros::NodeHandle n; 
  // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); \

  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;

  string traj_f("/home/david/.ros/vio/eit_f5_r2/eit_f5_r2_pvio_trajectory.log"); 
  string img_dir("/home/david/work/data/sr4k/imu_bdat/eit_f5_r2/sr4k"); 

  ros::NodeHandle np("~"); 
  np.param("pvio_traj_file", traj_f, traj_f); 
  np.param("img_directory", img_dir, img_dir); 

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
  
  // Load other trajectories 
  ros::NodeHandle np("~"); 
  string traj_dir("/home/david/.ros/vio/eit_f5_r2"); 
  np.param("trajectory_dir", traj_dir, traj_dir); 

  vector< vector<trajItem> > vtVIO; 
  // string vio5 = traj_dir + "/ba_5_vio_trajectory.log"; 
  // string vio10 = traj_dir + "/ba_10_vio_trajectory.log"; 
  // string vio20 = traj_dir + "/ba_20_vio_trajectory.log"; 
  // string vio30 = traj_dir + "/ba_30_vio_trajectory.log"; 
 
  string vio5 = traj_dir + "/ba_5_vio.ply"; 
  string vio10 = traj_dir + "/ba_10_vio.ply"; 
  string vio20 = traj_dir + "/ba_20_vio.ply"; 
  string vio30 = traj_dir + "/ba_30_vio.ply"; 
 

  vector<trajItem> tmp; 
  // if(!readTraj(vio5, tmp))
  if(!readTrajPly(vio5, tmp))
  {
    return false; 
  }
  vtVIO.push_back(tmp); tmp.clear(); 
  // if(!readTraj(vio10,tmp))
  if(!readTrajPly(vio10,tmp))
  {
    return false; 
  }
  vtVIO.push_back(tmp); tmp.clear(); 
  // if(!readTraj(vio20, tmp))
  if(!readTrajPly(vio20, tmp))
  {
    return false; 
  }
  vtVIO.push_back(tmp); tmp.clear(); 
  // if(!readTraj(vio30, tmp))
  if(!readTrajPly(vio30, tmp))
  {
    return false; 
  }
  vtVIO.push_back(tmp); tmp.clear(); 
  
  COLOR vcVIO[4]; 
  vcVIO[0] = BLUE;  // Tg = 5 blue 
  vcVIO[1] = RED;   // Tg = 10
  vcVIO[2] = PURPLE; // Tg = 20 
  vcVIO[3] = GREEN; // Tg = 30
  
  vector<string> vsId(4); 
  vsId[0] = "Tg_5"; vsId[1] = "Tg_10"; vsId[2] = "Tg_20"; vsId[3] = "Tg_30"; 

  CloudPtr pc_w(new Cloud); 
  cv::Mat i_img, d_img, df_img;   
  CSReadCV r4k; 
  
  // transform from imu to camera 
  Pose3 Pu2c; 
  setTu2c(Pu2c); 
    
  CVTKViewer<pcl::PointXYZRGBA> v;
  v.getViewer()->addCoordinateSystem(1.0, "cloud", 0); 
  v.getViewer()->setBackgroundColor(0.0, 0., 0.1, 0.7); 
  v.getViewer()->setSize(1000, 700); 
  v.addPointCloud(pc_w, "pwi"); 
  v.addPointCloud(pc_w, "pw"); 

  // yellow  
  // double r = 1.0; 

  for(int i=0; i<vt.size() && !v.stopped(); i++)
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
    // Point3 up_pc(0, -1, 0);   // (0, -1, 0) camera up direction 
    // Point3 up_pc(0, 0, 1); 
    Point3 up_pc(0, -0.98, 0.199); // dot(up_pc, view_pc-pos_pc) = 0
    Point3 pos_pw = Pw2c * pos_pc; 
    Point3 view_pw = Pw2c * view_pc; 
    Point3 up_pw = Pw2c * up_pc; 
    up_pw = up_pw - pos_pw; 

    ROS_INFO("camera position: %f %f %f", pos_pw.x(), pos_pw.y(), pos_pw.z()); 
    ROS_INFO("view point: %f %f %f", view_pw.x(), view_pw.y(), view_pw.z()); 
    ROS_INFO("up view point: %f %f %f", up_pw.x(), up_pw.y(), up_pw.z()); 

    // get image 
    stringstream ss; 
    ss<<img_dir<<"/d1_"<<setfill('0')<<setw(7)<<ti.sid<<".bdat";  // setw(4)
    r4k.readOneFrameCV(ss.str(), i_img, d_img);

    d_img.convertTo(df_img, CV_32FC1, 0.0001, 0); 
    // show images 
    cv::imshow("intensity img", i_img); 
    cv::imshow("depth img", df_img); 
    if(i==0)
    {
      cv::waitKey(0);
    }
    else{
      cv::waitKey(3); 
    }

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
    filterPointCloud<pcl::PointXYZRGBA>(0.10, pc_w, tmp); 
    pc_w.swap(tmp); 

    // set pose of the current point cloud 
     Eigen::Matrix<float, 4,4> Tf = T.cast<float>(); 
     Eigen::Affine3f Tf3(Tf); 
     // pwi->sensor_origin_ = Tf.block<4,1>(0,3); 
     // pwi->sensor_orientation_ = Eigen::Quaternionf(ti.qw, ti.qx, ti.qy, ti.qz); 

    const int K = 1; 
    if(!v.stopped())
    {
      // add trajectory 
      if(i % K == 0 && i >= K)
      {
        trajItem& i1 = vt[i-K]; 
        trajItem& i2 = vt[i]; 
        stringstream ss; 
        ss<<"pvio_" << i;
  
        addLine(v, i1.px, i1.py, i1.pz, i2.px, i2.py, i2.pz, YELLOW, ss.str()); 

        // add other trajectories for comparison 
        for(int j = 0; j<vtVIO.size(); j++)
        {
          vector<trajItem> & vT = vtVIO[j]; 
          if(i<vT.size() && i-K >= 0)
          {
            trajItem& j1 = vT[i-K]; 
            trajItem& j2 = vT[i]; 
            stringstream ss; 
            ss<<vsId[j]<<i; 
            addLine(v, j1.px, j1.py, j1.pz, j2.px, j2.py, j2.pz, vcVIO[j], ss.str()); 
          }
        }
      }

      // show the point cloud 
      // v.getViewer()->removeCoordinateSystem(); 
      // v.runOnce(1); 
      v.getViewer()->updatePointCloud(pwi, "pwi"); 
      v.getViewer()->updatePointCloud(pc_w, "pw");
      // v.getViewer()->resetCameraViewpoint("pwi"); 
      // v.getViewer()->setCameraPosition(ti.px, ti.py, ti.pz, view_pj.x(), view_pj.y(), view_pj.z(), up_pj.x(), up_pj.y(), up_pj.z());
      v.getViewer()->setCameraPosition(pos_pw.x(), pos_pw.y(), pos_pw.z(), view_pw.x(), view_pw.y(), view_pw.z(), up_pw.x(), up_pw.y(), up_pw.z());
      // v.getViewer()->addCoordinateSystem(0.2, Tf3);
      v.runOnce(10); 
      usleep(20*1000); 
    }
  }
    
  // 
  moveCameraInArc(v); 

  // wait for quit
  while(!v.stopped())
  {
   //   
    v.runOnce(); 
    usleep(100*1000); 
  }
  

  
return true; 
}

void moveCameraInArc(CVTKViewer<pcl::PointXYZRGBA>& v)
{
  // write a camera trajectory 
  int N = 50; 
  double r = 70.;
  double theta = M_PI/6.; 
  Point3 Up_PS(0, 1, 0); 
  Point3 PS(24, 0, -r); 
  Point3 PE(r*cos(theta), -r*sin(theta), 0.);
  double dtheta = M_PI/4./(N-1.);
  double px, py, pz, upx, upy, upz;  

  // for(int i=0; i<N/2; i++)
  int i = 0; 
  {
    pz = -cos(i * dtheta)*r; 
    py = -sin(i * dtheta)*r; 
    px = PS.x(); 
    ROS_WARN("set camera position: %f %f %f", px, py, pz); 
    
    upy = cos(i* dtheta); 
    upz = -sin(i* dtheta); 

    if(!v.stopped())
    {
      v.getViewer()->setCameraPosition(px, py, pz, PS.x(), PS.y(), 0., Up_PS.x(), upy, upz);
      v.runOnce(5); 
      // usleep(30*1000); 
    }
  }

 
}


void addLine( CVTKViewer<pcl::PointXYZRGBA>& v, float sx, float sy, float sz, float ex, float ey, float ez, COLOR c, string id)
{
  pcl::PointXYZRGBA p1, p2; 
  p1.x = sx; p1.y = sy; p1.z = sz; 
  p2.x = ex; p2.y = ey; p2.z = ez; 

  v.getViewer()->addLine(p1, p2, g_color[c][0]/255., g_color[c][1]/255., g_color[c][2]/255., id); 
}

bool readTrajPly(std::string f, vector<struct trajItem>& t)
{
  ifstream inf(f.c_str()); 
  if(!inf.is_open())
  {
    cout<<"failed to open file : "<<f<<endl;
    return false;
  }

  char buf[4096]; 
  for(int i=0; i<10; i++) // get rid of the first 10 lines 
  {
    inf.getline(buf, 4096);
  }

  while(inf.getline(buf, 4096))
  {
    trajItem ti; 
    // sscanf(buf, "%d %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.roll, &ti.pitch, &ti.yaw, &ti.sid); 
    // sscanf(buf, "%d %f %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw, &ti.sid); 
    sscanf(buf, "%f %f %f", &ti.px, &ti.py, &ti.pz);
    t.push_back(ti); 
  }

  cout << " read "<<t.size()<<" trajectory items"<<endl;
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




