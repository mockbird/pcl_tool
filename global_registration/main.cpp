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

#include "pair_registration.h"
// #include <boost/graph/graph_concepts.hpp>

const float GRID_SIZE=0.05;
const float NORMALS_RADIUS = 0.2;
//const float FEATURES_RADIUS = 0.2;
const float MAX_ICP_ITERATION = 30;

pcl::PointCloud<pcl::PointXYZ>::Ptr pairMerge (pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
  //　体素格　下采样降低数量
  //pcl::console::print_highlight ("Downsampling...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxelFilter(source,source_filtered,GRID_SIZE);
  voxelFilter(target,target_filtered,GRID_SIZE);
  
  //calculate normal
  pcl::PointCloud<pcl::PointNormal>::Ptr source_normal(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr target_normal(new pcl::PointCloud<pcl::PointNormal>);
  source_normal = getNormalcloud(source_filtered,NORMALS_RADIUS);
  target_normal = getNormalcloud(target_filtered,NORMALS_RADIUS);
  
  //
  //std::cout << "compute ICP" << std::endl;
  Eigen::Matrix4f icp_transform = icp(source_normal,target_normal,MAX_ICP_ITERATION);
  

  //ICP transform
  //std::cout << icp_transform << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_align(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*source,*source_align,icp_transform);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr merge(new pcl::PointCloud<pcl::PointXYZ>);

  *merge = *source_align + *target;
  viewPair(source,target,merge,target);
  return merge;
}

int main(int argc, char **argv) {

  std::cout << "Loading clouds" << std::endl;
  
  // load point cloud files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr merge(new pcl::PointCloud<pcl::PointXYZ>);
  
  std::cout << "loading file " << argv[1] << std::endl;
  pcl::io::loadPCDFile (argv[1], *source);
  
  
  for (int i = 2; i < argc; i++) //遍历所有的文件名（略过程序名）
  {
    std::string fname = std::string (argv[i]);
    std::cout << "registration with " << fname << std::endl;
    pcl::io::loadPCDFile (argv[i], *target);
    
    merge = pairMerge(source,target);
    
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setLeafSize(0.01, 0.01, 0.01);
    vox_grid.setInputCloud(merge);
    vox_grid.filter(*source);
    //*source = *merge;
  }
  
  pcl::io::savePCDFileBinary("/home/user/data/train/merge.pcd",*source);
  
  return 0;
}

// /home/user/projects/sac-icp2.0/build/sac-icp2.0  /home/user/data/registration/source.pcd /home/user/data/registration/target.pcd