/*
 *  Nov. 28 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  record a video of reconstructing the 3D map, given the trajectory and point clouds 
 *  using realsense camera
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
#include <opencv/cv.h>
#include "glob_def.h"
// #include "pc_from_image.h"
// #include "SR_reader_cv.h"
#include "vtk_viewer.h"

using namespace std; 
using namespace gtsam;

struct trajItem
{
  string timestamp; 
  int id; int sid; // sequence id 
  float px, py, pz; // euler angle roll, pitch, yaw; 
  float qx, qy, qz, qw; // quaternion
};

struct obsItem
{
  struct trajItem pose; 
  string frgb; // rgb's filename 
  string fdpt; // dpt's filename
};

// 1. read traj
bool readTraj(std::string f, vector<struct trajItem>& );
// 2. load camera images
bool loadImgFiles(vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt, vector<string>& mv_ir1, vector<string>& mv_ir2);
// 3. associate these two together 
bool associateItem(vector<obsItem>&, vector<trajItem>& , vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt);

// 4. draw functions 
void generatePointCloud(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, int skip =1);
bool showPCDIncremental(); 
void addLine( CVTKViewer<pcl::PointXYZRGBA>& , float sx, float sy, float sz, float ex, float ey, float ez, COLOR c, string id); 
void moveCameraInArc(CVTKViewer<pcl::PointXYZRGBA>& v); 
void setTu2c(Pose3& Tu2c);

template<typename PointT>
void filterPointCloud(float _voxel_size, typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out)
{
  // voxelgrid filter
  pcl::VoxelGrid<PointT> vog;
  vog.setInputCloud(in); 
  vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
  vog.filter(*out);
}

string g_img_dir = "";
string g_traj_f = "";

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map_video_rs"); 
    ros::NodeHandle n; 
    // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); 

    cout<<"usage: ./map_video img_dir trajectory_file"<<endl; 

    if(argc >= 2)
	g_img_dir = argv[1]; 
    if(argc >= 3)
	g_traj_f = argv[2]; 
    
    ROS_WARN("map_video_rs: g_img_dir = %s", g_img_dir.c_str()); 
    ROS_WARN("map_video_rs: g_traj_f = %s", g_traj_f.c_str()); 

    showPCDIncremental(); 

    return 1; 
}


bool showPCDIncremental()
{
    string traj_f = g_traj_f;
    string img_dir = g_img_dir; 

    // 1. load trajectory 
    vector<trajItem> vts; 
    if(!readTraj(traj_f, vts))
    {
	ROS_ERROR("map_video_rs: failed to read Trajectory!");
	return false; 
    }

    // 2. load img file names 
    vector<string> mv_rgb; 
    vector<string> mv_timestamp;
    vector<string> mv_dpt; 
    vector<string> mv_ir1; 
    vector<string> mv_ir2; 
    if(!loadImgFiles(mv_timestamp, mv_rgb, mv_dpt, mv_ir1, mv_ir2)) 
    {
	ROS_ERROR("map_video_rs: failed to read camera data!"); 
	return false;
    }

    // 3. associate these two to generate point cloud at poses 
    vector<obsItem> vobs; 
    if(!associateItem(vobs, vts, mv_timestamp, mv_rgb, mv_dpt))
    {
	ROS_ERROR("map_video_rs: failed to associate camera datan with trajectory data!");
	return false; 
    }

    // 4. draw them incrementally 
    CloudPtr pc_w(new Cloud); 
    cv::Mat i_img, d_img;   
    // CSReadCV r4k; 

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
    bool waitForStart = true; 

    for(int i=0; i<vobs.size() && !v.stopped(); i++)
    {
	obsItem& obi = vobs[i]; 
	trajItem& ti = obi.pose; 

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

	Point3 pos_pc(0, -1, -2.5); // (0, -2, -5) // camera pose
	Point3 view_pc(0, 0, 2.5);  // (0, 0, 5) camera view point 
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
	// ss<<img_dir<<"/d1_"<<setfill('0')<<setw(7)<<ti.sid<<".bdat";  // setw(4)
	// r4k.readOneFrameCV(ss.str(), i_img, d_img);
	string frgb = g_img_dir + "/" + obi.frgb; 
	string fdpt = g_img_dir + "/" + obi.fdpt;
	
	i_img = cv::imread(frgb.c_str(), -1); 
	d_img = cv::imread(fdpt.c_str(), -1); 

	// show images 
	cv::namedWindow("intensity img", CV_WINDOW_KEEPRATIO); 
	cv::namedWindow("depth img", CV_WINDOW_KEEPRATIO); 
	cv::imshow("intensity img", i_img); 
	cv::imshow("depth img", d_img); 
	cv::waitKey(3); 
	if(waitForStart)
	{
	    cout <<"wait for start!" <<endl; 
	    int k; 
	    cin >> k; 
	    waitForStart = false; 
	}

	// point cloud 
	CloudPtr pci(new Cloud);
	CloudPtr pwi(new Cloud);
	// generatePointCloud(i_img, d_img, 0.001, cam, *pci); 
	generatePointCloud(i_img, d_img, pci); 

	Eigen::Matrix4d T = Pw2c.matrix();  
	// pcl::transformPointCloud(*pci, *pwi, Tw2j.cast<float>()); // transform into global 
	pcl::transformPointCloud(*pci, *pwi, T.cast<float>()); // transform into global 

	// add to global point cloud
	*pc_w += *pwi; 

	// voxel grid filter
	CloudPtr tmp(new Cloud); 
	// filterPointCloud<pcl::PointXYZRGBA>(0.1, pc_w, tmp); 
	filterPointCloud<pcl::PointXYZRGBA>(0.05, pc_w, tmp); 
	pc_w.swap(tmp); 

	// set pose of the current point cloud 
	Eigen::Matrix<float, 4,4> Tf = T.cast<float>(); 
	Eigen::Affine3f Tf3(Tf); 
	// pwi->sensor_origin_ = Tf.block<4,1>(0,3); 
	// pwi->sensor_orientation_ = Eigen::Quaternionf(ti.qw, ti.qx, ti.qy, ti.qz); 

	const int K = 3; // 1 
	if(!v.stopped())
	{
	    // add trajectory 
	    if(i % K == 0 && i >= K)
	    {
		// trajItem& i1 = vt[i-K]; 
		// trajItem& i2 = vt[i]; 
		trajItem& i1 = vobs[i-K].pose;
		trajItem& i2 = vobs[i].pose;
		stringstream ss; 
		ss<<"pvio_" << i;

		addLine(v, i1.px, i1.py, i1.pz, i2.px, i2.py, i2.pz, YELLOW, ss.str()); 
	    }

	    // show the point cloud 
	    v.getViewer()->removeCoordinateSystem(); 
	    // v.runOnce(1); 
	    v.getViewer()->updatePointCloud(pwi, "pwi"); 
	    v.getViewer()->updatePointCloud(pc_w, "pw");
	    // v.getViewer()->resetCameraViewpoint("pwi"); 
	    // v.getViewer()->setCameraPosition(ti.px, ti.py, ti.pz, view_pj.x(), view_pj.y(), view_pj.z(), up_pj.x(), up_pj.y(), up_pj.z());
	    v.getViewer()->setCameraPosition(pos_pw.x(), pos_pw.y(), pos_pw.z(), view_pw.x(), view_pw.y(), view_pw.z(), up_pw.x(), up_pw.y(), up_pw.z());
	    v.getViewer()->addCoordinateSystem(0.2, Tf3);
	    v.runOnce(30); 
	    usleep(30*1000); 
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

  for(int i=0; i<N/2; i++)
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

bool readTraj(std::string f, vector<struct trajItem>& t)
{

  ifstream inf(f.c_str()); 
  if(!inf.is_open())
  {
    cout<<"failed to open file : "<<f<<endl;
    return false;
  }

//  char buf[4096]; 
//  while(inf.getline(buf, 4096))
//  {
//    trajItem ti; 
//    // sscanf(buf, "%d %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.roll, &ti.pitch, &ti.yaw, &ti.sid); 
//    sscanf(buf, "%d %f %f %f %f %f %f %f %d", &ti.id, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw, &ti.sid); 
//    t.push_back(ti); 
//  }
//
//  cout << " read "<<t.size()<<" trajectory items"<<endl;
//  return true;
//
 char buf[4096]; 

  while(inf.getline(buf, 4096))
  {
    char* p = strtok(buf, " ,"); 
    if(p == NULL)
    {
      break; 
    }
    trajItem ti;
    ti.timestamp = string(p).substr(0, 10) + "." + string(p).substr(10, 4); // here, discard values < e-5 second
    p = strtok(NULL, " ,");  ti.px = atof(p); 
    p = strtok(NULL, " ,");  ti.py = atof(p); 
    p = strtok(NULL, " ,");  ti.pz = atof(p); 
    p = strtok(NULL, " ,");  ti.qw = atof(p); 
    p = strtok(NULL, " ,");  ti.qx = atof(p); 
    p = strtok(NULL, " ,");  ti.qy = atof(p); 
    p = strtok(NULL, " ,");  ti.qz = atof(p); 
    t.push_back(ti);
  }

  ROS_INFO("map_video_rs: obtain %d items", t.size()); 
  return true; 
}

bool loadImgFiles(vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt, vector<string>& mv_ir1, vector<string>& mv_ir2)
{
  ifstream fAssociation;
  string strAssociationFilename = g_img_dir + "/timestamp.txt"; 
  fAssociation.open(strAssociationFilename.c_str());
  if(!fAssociation.is_open())
  {
    ROS_ERROR("map_video_rs: failed to open file %s", strAssociationFilename.c_str()); 
    return false; 
  }
  mv_rgb.clear(); 
  mv_dpt.clear(); 
  mv_ir1.clear(); 
  mv_ir2.clear(); 
  mv_timestamp.clear(); 

  string s;
  getline(fAssociation,s); // delete the first line  

  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation,s);// 
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      string t; 
      string sRGB, sD, sr1, sr2;
      ss >> t;
      mv_timestamp.push_back(t.substr(0,15));
      ss >> sRGB;
      mv_rgb.push_back(sRGB);
      ss >> t;
      ss >> sD;
      mv_dpt.push_back(sD);
      ss >> t; 
      ss >> sr1; 
      mv_ir1.push_back(sr1); 
      ss >> t; 
      ss >> sr2; 
      mv_ir2.push_back(sr2); 
    }
  }
  
  ROS_INFO("map_video_rs: succeed to load %d img file names", mv_timestamp.size());
  
  return true; 
}

void generatePointCloud(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, int skip ) 
{  
  double z; 
  double px, py, pz; 
  int height = rgb.rows/skip; 
  int width = rgb.cols/skip; 
  int N = (rgb.rows/skip)*(rgb.cols/skip); 
  // CamModel rs_R200(615.426, 625.456, 318.837, 240.594); 
  double fx = 615.426; 
  double fy = 625.456; 
  double cx = 318.837;
  double cy = 240.594;

  pc->points.reserve(N); 
  // pc.width = width; 
  // pc.height = height; 

  unsigned char r, g, b; 
  int pixel_data_size = 3; 
  if(rgb.type() == CV_8UC1)
  {
    pixel_data_size = 1; 
  }
  
  int color_idx; 
  char red_idx = 2, green_idx =1, blue_idx = 0;

  Point pt; 

  for(int v = 0; v<rgb.rows; v+=skip)
  for(int u = 0; u<rgb.cols; u+=skip)
  {
    // Point& pt = pc.points[v*width + u]; 
    z = dpt.at<float>((v), (u));
    if(std::isnan(z) || z <= 0.1 || z >= 5) 
    {
      continue; 
    }

    // compute point 
    pz = z; 
    px = ((u-cx)/fx)*pz;
    py = ((v-cy)/fy)*pz;

    pt.x = px;  pt.y = py;  pt.z = pz; 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    pt.r = r; pt.g = g; pt.b = b; 
    pc->points.push_back(pt); 
  }
  pc->height = 1; 
  pc->width = pc->points.size(); 
  return ;
}

inline bool sameTime(string t1, string t2)
{
    double dt1 = atof(t1.c_str());
    double dt2 = atof(t2.c_str());
    if(fabs(dt1-dt2) < 1e-3)
	return true;
    return false;
}


bool associateItem(vector<obsItem>& vobs, vector<trajItem>& vts, vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt)
{
  // test 
  // ofstream ouf("associate_file_name_result.log"); 
  
  for(int i=0, lj = 0; i<vts.size(); i++)
  {
    string ti = vts[i].timestamp; 
    int j = lj; 
    obsItem obs; 
    obs.pose = vts[i]; 
    for(; j<mv_timestamp.size(); j++)
    {
      string tj = mv_timestamp[j]; 
      // if(tj == ti)
      if(sameTime(ti, tj))
      {
        obs.frgb = mv_rgb[j]; 
        obs.fdpt = mv_dpt[j]; 
        // ouf <<ti<<"\t"<<obs.frgb<<"\t"<<obs.fdpt<<endl;
        break; 
      }
    }
    lj = j; 
    if(j == mv_timestamp.size())
    {
      ROS_ERROR("vins-mapping: j = mv_timestamp.size() something is wrong!");
      return false; 
    }
    vobs.push_back(obs);
  }
  ROS_INFO("map_video_rs: succeed to associate %d obsItems", vobs.size()); 
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

  // make it as Identity 
  Rot3 R_zero = Rot3::RzRyRx(0, 0, 0); 
  Tu2c = Pose3::Create(R_zero, t); 

   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<Tu2c*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 

  return ; 
}




