/*
 * Jan. 11, 2017, He Zhang 
 *
 * Generate 3D map: 
 *     input:  a trajectory.log and img dir where the images are stored 
 *     output: a .ply file, shown the 3d point cloud 
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
#include "sparse_feature_vo.h"
#include "cam_model.h"
#include "SR_reader_cv.h"

using namespace gtsam; 
using namespace std; 

#define D2R(d) (((d)*M_PI)/180)

string fname("/home/david/work/data/sr4k/imu_bdat/etas_f5/imu_v100.log"); 

struct trajItem
{
  int id; int sid; // sequence id 
  float px, py, pz; // euler angle roll, pitch, yaw; 
  float qx, qy, qz, qw; // quaternion
};

void setTu2c(Pose3& Tu2c); 
void headerPLY(std::ofstream& ouf, int vertex_number);
bool mapPLY(std::string f, std::string img_dir, std::string outPLY, int skip, float depth_scale, CSparseFeatureVO* pSF, int* rect = 0);
bool readTraj(std::string f, vector<struct trajItem>& );

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mapping_PLY"); 
  ros::NodeHandle n; 
  // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); \

  CamModel sr4k(250.5773, 250.5773, 90, 70, -0.8466, 0.5370); 
  sr4k.z_offset = 0.015;  // this is only for sr4k 
  float depth_scale = 0.001;
  CSparseFeatureVO spr_vo(sr4k);

  // parameters 
  ros::NodeHandle np("~"); 
  int skip = 2; 
  string trajFile;  // the trajectory log file 
  string imgDir;    // images dir 
  string outPLY;    // the output PLY file 
  
  int su, sv, eu, ev ; 

  np.param("trajectory_file", trajFile, trajFile); 
  np.param("img_directory", imgDir, imgDir); 
  np.param("output_PLY_file", outPLY, outPLY); 
  np.param("downsample_skip", skip, skip); 
  np.param("top_left_u", su, 0); 
  np.param("top_left_v", sv, 0); 
  np.param("bot_right_u", eu, 176); 
  np.param("bot_right_v", ev, 144); 
  

  int rect[4] = {su, sv, eu, ev}; 

  cout <<"rect "<<rect[0]<<" "<<rect[1]<<" "<<rect[2]<<" "<<rect[3]<<endl;

  mapPLY(trajFile, imgDir, outPLY, skip, depth_scale, &spr_vo, &rect[0]);

  ROS_INFO("Finish mapping into %s!", outPLY.c_str());

  return 0; 
}

bool mapPLY(std::string f, std::string img_dir, std::string outPLY, int skip, float depth_scale, CSparseFeatureVO* pSF, int* rect)
{
  // output file 
  ofstream ouf(outPLY.c_str()); 
  if(!ouf.is_open())
  {
    cout <<" failed to open output PLY file : "<<outPLY<<endl; 
    return false; 
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
  CSReadCV r4k; 

  // transform point from camera to imu 
  Pose3 Tu2c ; 
  setTu2c(Tu2c); 

  for(int i=0; i<t.size()-20; i++)
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
    stringstream ss; 
    ss<<img_dir<<"/d1_"<<setfill('0')<<setw(7)<<ti.sid<<".bdat";  // setw(4)
    r4k.readOneFrameCV(ss.str(), i_img, d_img);
    // compute local point cloud 
    vector<float> p_loc; 
    vector<unsigned char> p_col; 
    pSF->generatePointCloud(i_img, d_img, skip, depth_scale, p_loc, p_col, rect); 

    // transform into global 
    Point3 fo, to, to_imu; 
    for(int i=0; i<p_loc.size(); i+=3)
    {
      fo << p_loc[i], p_loc[i+1], p_loc[i+2]; 
      to_imu = Tu2c.transform_from(fo);
      to = p.transform_from(to_imu); 
      p_loc[i] = to(0); p_loc[i+1] = to(1); p_loc[i+2] = to(2); 
    }
    
    pts_loc.insert(pts_loc.end(), p_loc.begin(), p_loc.end()); 
    pts_col.insert(pts_col.end(), p_col.begin(), p_col.end());

    // ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<g_color[c][0]<<" "<<g_color[c][1]<<" "<<g_color[c][2]<<endl;
    ROS_INFO("handle %d frame, add %d pts", ti.sid, p_loc.size());
  }
  
  // output into file 

  // first, add header 
  
  int vertex_number = pts_loc.size()/3; 
  headerPLY(ouf, vertex_number);

  for(int i=0; i<pts_loc.size(); i+=3)
  {
    ouf<< pts_loc[i]<<" "<<pts_loc[i+1]<<" "<<pts_loc[i+2]<<" "<<(int)pts_col[i]<<" "<<(int)pts_col[i+1]<<" "<<(int)pts_col[i+2]<<endl;
  }
  ouf.flush(); 
  ouf.close(); 
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
