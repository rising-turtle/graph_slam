/*
    July. 23 2018, He Zhang, hzhang8@vcu.edu 

    input a pcd, convert it to pointcloud2 for publication 
*/

#include <iostream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace std; 

void pub_pcd(string pcd_file); 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pub_pcd"); 
    ros::NodeHandle n; 
    if(argc < 2) 
    {
	cout <<"usage: pub_pcd *.pcd"<<endl; 
	return 1;
    }
    
    string pcd_file = string(argv[1]); 
    pub_pcd(pcd_file); 
    return 1; 
}

void pub_pcd(string pcd_file)
{
    // 1. read pcd file 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::io::loadPCDFile(pcd_file, *pc); 

    pc->width = pc->points.size(); 
    pc->height = 1; 
    pc->is_dense = true; 
 
    // 2. convert to pointcloud2 
    sensor_msgs::PointCloud2 out_pc; 
    pcl::toROSMsg(*pc, out_pc); 
    out_pc.header.frame_id = "map"; 

    // 3. publish it 
    
    //! @note publish loop
    ros::Rate loop_rate(1.);
    ros::NodeHandle nh; 
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_map", 1);
    while (ros::ok())
    {
	pcl_pub.publish(out_pc);
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;

}
