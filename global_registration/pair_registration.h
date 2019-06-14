#include <iostream>
#include <time.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/console/time.h>
#include <pcl/correspondence.h>

void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize){
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setLeafSize(gridsize, gridsize, gridsize);
  vox_grid.setInputCloud(cloud_in);
  vox_grid.filter(*cloud_out);
  return;
}

pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, double normals_radius) {
  pcl::PointCloud<pcl::Normal>::Ptr normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> norm_est;
  norm_est.setInputCloud(incloud);
  norm_est.setRadiusSearch(normals_radius);
  norm_est.compute(*normalsPtr);
  return normalsPtr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr getNormalcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, double normals_radius)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> norm_est;
  norm_est.setInputCloud(incloud);
  norm_est.setRadiusSearch(normals_radius);
  norm_est.compute(*cloud_normals);
  pcl::copyPointCloud (*incloud, *cloud_normals); 
  return cloud_normals;
}

Eigen::Matrix4f icp(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target,
	int MAX_ICP_ITERATION){
  
  
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal,pcl::PointNormal> icp;

  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_target);
  icp.setMaximumIterations(MAX_ICP_ITERATION);
  icp.setMaxCorrespondenceDistance(0.1);
  icp.setTransformationEpsilon(1e-6);
  pcl::PointCloud<pcl::PointNormal> Final;         //存储经过配准变换点云后的点云
  icp.align(Final);  
  Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
  return icp_transform;
}

// Eigen::Matrix4f icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
// 	int MAX_ICP_ITERATION){
//   pcl::IterativeClosestPointNonLinear<pcl::PointXYZ,pcl::PointXYZ> icp;
// 
//   icp.setInputSource(cloud_source);
//   icp.setInputTarget(cloud_target);
//   icp.setMaximumIterations(MAX_ICP_ITERATION);
//   icp.setMaxCorrespondenceDistance(0.1);
//   icp.setTransformationEpsilon(1e-6);
//   pcl::PointCloud<pcl::PointXYZ> Final;         //存储经过配准变换点云后的点云
//   icp.align(Final);  
//   Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
//   return icp_transform;
// }

void viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->initCameraParameters();
  viewer->setBackgroundColor(0,0,255,0);
  viewer->addCoordinateSystem (1.0);

// 设置相机的坐标和方向
  viewer->setCameraPosition (-3.68332/2, 2.94092/2, 5.71266/2, 0.289847, 0.921947, -0.256907, 0);
  viewer->setSize (1280, 1024);// 可视化窗口的大小
  
  int v1(0), v2(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
//   viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
//   viewer->initCameraParameters();   //初始化相机参数
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud1, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);
  viewer->addPointCloud(cloud1, green, "v1_target", v1);
  viewer->addPointCloud(cloud2, red, "v1_sourse", v1);


  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
  viewer->addText("After Alignment", 10, 10, "v2 text", v2);
//   viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green2(cloud1al, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red2(cloud2al, 255, 0, 0);
  viewer->addPointCloud(cloud1al, green2, "v2_target", v2);
  viewer->addPointCloud(cloud2al, red2, "v2_sourse", v2);
  viewer->spin();

  return;
}

