#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/visualization/cloud_viewer.h>

void viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->initCameraParameters();

  int v1(0), v2(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
//   viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
  viewer->initCameraParameters();   //初始化相机参数
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

int main(int argc, char **argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bim(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>());
  
  // 载入物体点云　和　场景点云　object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPLYFile ("/home/user/data/registration/cloud_scene.ply", *cloud_scene) < 0 ||
      pcl::io::loadPLYFile ("/home/user/data/registration/cloud_bim.ply", *cloud_bim) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }
  
  pcl::PointCloud<pcl::PointXYZ> registrationPoints;
  pcl::PointCloud<pcl::PointXYZ> registrationPointsBim;
  pcl::PointCloud<pcl::PointXYZ> registrationPointsScene;
  
  pcl::io::loadPCDFile("/home/user/data/registration/registrition_points.pcd",registrationPoints);
  std::cout << registrationPoints.points.size() << std::endl;
  for (size_t i=0;i<3;i++)
  {
    registrationPointsBim.push_back(registrationPoints[i]);
    registrationPointsScene.push_back(registrationPoints[i+3]);
  }

      
  pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ,pcl::PointXYZ,float> TESVD;
  pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ,pcl::PointXYZ,float>::Matrix4 transformation;
  TESVD.estimateRigidTransformation (registrationPointsBim,registrationPointsScene,transformation);
  
  std::cout << transformation << std::endl;
  
  pcl::transformPointCloud(*cloud_scene,*cloud_align,transformation);

  viewPair(cloud_bim,cloud_scene,cloud_bim,cloud_align);

  std::cout << "Hello, world!" << std::endl;
  return 0;
}
