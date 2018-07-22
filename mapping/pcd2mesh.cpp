#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <cstdio>
#include <ctime>

typedef pcl::PointXYZRGB  Point;
typedef pcl::PointCloud<Point>  Cloud;  
typedef typename pcl::PointCloud<Point>::Ptr CloudPtr;

int main (int argc, char** argv)
{
  std::clock_t start;
  double duration;

  start = std::clock();

  std::string output_name = "mesh.ply"; 
  if(argc < 2)
  {
    cout <<"usage: pcd2mesh *.pcd [mesh.ply]"<<endl; 
    return ; 
  }
  if(argc >=3)
    output_name = std::string(argv[2]); 
    
  // Load input file into a PointCloud<T> with an appropriate type
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  CloudPtr cloud(new Cloud); 
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (argv[1], cloud_blob);

  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

  cout << cloud->points.size () << " points"<<endl;
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<Point, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Finish

   // pcl::io::saveVTKFile ("mesh.vtk", triangles);
   
   pcl::io::savePLYFile("mesh.ply", triangles); 
//
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   viewer->addPolygonMesh(triangles,"meshes",0);
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   while (!viewer->wasStopped ()){
//       viewer->spinOnce (100);
//       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//   }
//
//
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

  std::cout<<"converting time:: "<< duration <<'\n';

  return (0);
}
