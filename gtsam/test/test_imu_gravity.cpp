/*
    July 23rd 2018, He Zhang, hzhang8@vcu.edu 

    test the gravity direction using imu 

*/

#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <pcl/common/transforms.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/geometry/Pose3.h>
#include "sparse_feature_vo.h"
#include "camera_node.h"
#include "cam_model.h"
#include "gtsam_graph.h"
#include "pc_from_image.h"
#include "rs_r200_wrapper.h"
#include "imu_vn100.h"
#include "chi2.h"
#include "plane.h"
#include "vtk_viewer.h"

using namespace std; 
using namespace gtsam;

void test_gravity(); 

int main(int argc, char* argv[])
{
    test_gravity(); 
    return 0; 
}

void test_gravity()
{
    // 1. read camera timestamp 
    string data_dir("/home/davidz/work/data/drone/static_2"); 
    string img_time_file = data_dir + "/timestamp.txt"; 
    double t; 
    string rgb_file; 
    string dpt_file; 
    {
	ifstream inf(img_time_file.c_str()); 
	if(!inf.is_open())
	{
	    cout <<"test_imu_gravity.cpp: failed to red file "<<img_time_file<<endl;
	    return ; 
	}
	string line;
	getline(inf, line); 
	stringstream ss; 
	ss << line; 
	ss >> t; 
	ss >> rgb_file; 
	ss >> t; 
	ss >> dpt_file;
    }

    // 2. read imu data 
    string imu_file = data_dir + "/imu_vn100_even.log"; 
    gtsam::imuBias::ConstantBias prior_bias; 
    double dt = 0.005; // 200 hz
    CImuVn100* imu = new CImuVn100(dt, prior_bias); 
    if(!imu->readImuData(imu_file))
    {
	ROS_ERROR("failed to load imu data from file %s", imu_file.c_str()); 
	return ; 
    }

    // 3. camera model to read camera data and initialize gtsam
    CamModel rs435(617.306, 617.714, 326.245, 239.974); 
    CCameraNode::set_cam_cov(rs435);
    float depth_scale = 0.001;
    rs435.setDepthScale(depth_scale); 
    CamModel::updategCamModel(rs435); 
    // CSparseFeatureVO spr_vo(rs435);
    CRSR200Wrapper cam; 
    // CGraphGT gt_graph;
    cv::Mat i_img, d_img; // 
    rgb_file = data_dir + "/" + rgb_file; 
    dpt_file = data_dir + "/" + dpt_file; 
    if(!cam.readOneFrameCV(rgb_file, dpt_file, i_img, d_img))
    {
	cout<<"test_imu_gravity.cpp: failed to load rgb_file: "<<rgb_file<<endl; 
	return ; 
    }

    // CCameraNode* pNewNode = new CCameraNode();
    // spr_vo.featureExtraction(i_img, d_img, depth_scale, *pNewNode); 
    // pNewNode->m_seq_id = 0; 
    // gt_graph.firstNode(pNewNode, false);
    imu->setStartPoint(t);  // first measurement 

    // 4. compute gravity direction 
    double ax, ay, az; 
    imu->getNormalizedAcc(ax, ay, az); 
    // gt_graph.initFromImu(ax, ay, az); 
    Eigen::Vector3d fv(ax, ay, az); 
    Eigen::Vector3d tv(0, 0, -1); 
    Eigen::Vector3d w = fv.cross(tv).normalized(); 
    double angle = acos(fv.dot(tv)); 
    
    double half_angle = angle /2.;
    Eigen::Vector4d vq; 
    vq.head<3>() = w * sin(half_angle); 
    vq[3] = cos(half_angle); 
    Eigen::Quaterniond q(vq); 
    Eigen::Matrix<double, 3, 3> m = q.toRotationMatrix(); 
    Eigen::Vector3d dst_t = m * fv; 
    Rot3 R(m);
    Point3 tm(0, 0, 0); 
    Pose3 new_pose(R,tm);    

    cout <<"noramlzied a = "<<ax<<", "<<ay<<", "<<az<<endl; 
    cout <<"dst_t = "<<dst_t<<endl; 

    // 5. transform point cloud for viewing
    CloudPtr pc(new Cloud); 
    generatePointCloud(i_img, d_img, 0.001, rs435, *pc); 

    // 6. compute a plane for comparison 
    CPlane plane; 
    plane.computeParameters(pc); 
    Eigen::Vector3d ni(plane.nx_, plane.ny_, plane.nz_); 
    Eigen::Vector3d nw = m * ni; 
    cout <<"ni: "<<ni<<endl; 
    cout <<"nw: "<<nw<<endl; 

    angle = acos(ni.dot(fv)); 
    cout <<"angle between ni and ai: "<< angle*180./M_PI<<endl; 
    angle = acos(nw.dot(dst_t)); 
    cout <<"angle between nw and aw: "<< angle*180./M_PI<<endl;
    

    Eigen::Matrix4d T = new_pose.matrix();  // gt_graph.mp_w2o->matrix(); 
    CloudPtr pc_w(new Cloud); 
    pcl::transformPointCloud(*pc, *pc_w, T.cast<float>()); 

    markColor(*pc, plane.inliers_->indices, BLUE); 

    CVTKViewer<pcl::PointXYZRGBA> v;
    // CVTKViewer<pcl::PointXYZRGB> v;

    v.getViewer()->addCoordinateSystem(1.0, 0, 0, 0); 
    // v.addPointCloud(pc_w, "PC in world"); 
    v.addPointCloud(pc, "PC in camera");
    while(!v.stopped())
    {
	v.runOnce(); 
	usleep(100*1000); 
    }

    return ;
}
