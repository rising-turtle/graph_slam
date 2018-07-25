/*
    July 23 2019, He Zhang, hzhang8@vcu.edu  

    process the point cloud, e.g.
	- get rid of noisy point using cluster 
	- remove floor point using z value 
	- ...
*/

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
// #include "vtk_viewer.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std; 

typedef pcl::PointXYZRGB C_PT;
typedef pcl::PointCloud<C_PT> C_CD;
typedef pcl::PointCloud<C_PT>::Ptr CCD_PR; 

template<typename PointT>
void passThroughZ(float min_v, float max_v, string axis, typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out)
{
    pcl::PassThrough<PointT> pf; 
    pf.setInputCloud(in); 
    pf.setFilterFieldName(axis.c_str()); 
    pf.setFilterLimits(min_v, max_v); 
    pf.filter(*out); 
}

template<typename PointT>
void clusterFilter( double tol_dis, typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out)
{
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); 
    tree->setInputCloud(in); 

    vector<pcl::PointIndices> cluster_indices; 
    pcl::EuclideanClusterExtraction<PointT> ec; 
    ec.setClusterTolerance(tol_dis); 
    ec.setMinClusterSize(200); 
    // ec.setMaxClusterSize();
    ec.setSearchMethod(tree); 
    ec.setInputCloud(in); 
    ec.extract(cluster_indices); 
	
    out->points.clear(); 
    for(int i=0; i<cluster_indices.size(); i++)
    {
	for(int j=0; j<cluster_indices[i].indices.size(); j++)
	{
	    int index = cluster_indices[i].indices[j]; 
	    out->points.push_back(in->points[index]); 
	}
    }
    out->width = out->points.size(); 
    out->height = 1; 
    out->is_dense = true; 
    return ; 
}

void remove_floor_pt(CCD_PR& in, CCD_PR& out);

int main(int argc, char* argv[])
{
    // 1. read pcd file 
    if(argc < 2) 
    {
	cout <<"usage: pcd_filter *.pcd"<<endl; 
	return 1;
    }
    
    string pcd_file = string(argv[1]); 

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::io::loadPCDFile(pcd_file, *pc); 

    pc->width = pc->points.size(); 
    pc->height = 1; 
    pc->is_dense = true; 

    // 2. filter out noisy pt using cluster 
    CCD_PR pc1(new C_CD); 
    clusterFilter<pcl::PointXYZRGB>(0.03, pc, pc1); 
   
    // 3. remove floor data 
    CCD_PR pc2(new C_CD); 
    remove_floor_pt(pc1, pc2); 

    // 4. save the output pcd
    pcl::io::savePCDFile("filtered_pc.pcd", *pc2); 
    return 1;
}


void remove_floor_pt(CCD_PR& in, CCD_PR& out)
{
    // 1. find z range [min_z, max_z]
    double min_z = 1e10; 
    double max_z = -1e10; 
    
    for(int i=0; i<in->points.size(); i++)
    {
	C_PT& pt = in->points[i]; 
	if(min_z > pt.z ) min_z = pt.z; 
	if(max_z < pt.z ) max_z = pt.z; 
    }
    
    cout <<"pcd_filter.cpp : min_z: "<<min_z<<" max_z: "<<max_z<<endl; 
    
    // 2. histogram into different bins 
    double res = 0.1; 
    if(max_z - min_z > 2.0) max_z = min_z + 2.0; 
    int n_bins = (max_z - min_z)/res + 1; 
    
    cout <<"n_bins: "<<n_bins<<endl;
    map<int, vector<int> > bins; 

    for(int i=0; i<in->points.size(); i++)
    {
	C_PT& pt = in->points[i]; 
	int ind = (pt.z - min_z + res/2.)/res;
	bins[ind].push_back(i); 
    }
    
    // 3. find the bin with most points 
    int max_n = 0; 
    int max_id = -1; 
    map<int, vector<int> >::iterator it = bins.begin();

    while(it != bins.end())
    {
	// cout <<"bin: "<<it->first<<" has: "<<it->second.size()<<endl;
	if(it->second.size() > max_n) 
	{
	    max_n = it->second.size(); 
	    max_id = it->first; 
	}
	++it; 
    }
    float z_floor = min_z + max_id*res; 
    cout <<"max_id: "<<max_id<<" floor_z: "<<z_floor<<" max_n: "<<max_n<<endl; 

    // 4. filter out points in range [min_z, z_floor + flight_height(0.2) ] 
    out->points.clear(); 
    out->points.reserve(in->points.size()); 
    for(int i=0; i<in->points.size(); i++)
    {
	C_PT& pt = in->points[i]; 
	if(pt.z > z_floor + 0.1)
	{
	    pt.z = pt.z - z_floor; // take floor as the x-y axis
	    out->points.push_back(pt); 
	}
    }
    out->width = out->points.size(); 
    out->height = 1; 
    out->is_dense = true; 
    cout <<"out pc has "<<out->width<<" points!"<<endl;
    return ; 
}






