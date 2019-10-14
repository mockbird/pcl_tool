/*
*
* @file main.cpp
* @brief 点云和点云自动配准
* @author MiaoYan
* @email  miuy@glodon.com
*
*/

#include <iostream>              //标准C++库中的输入输出的头文件
#include <string>

//#include "pointcloud.h"
#include "pointcloud.h"

#include <pcl/io/pcd_io.h>

#include <pcl/filters/extract_indices.h>//按索引提取点云
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>

#include <pcl/console/time.h>


const float NORMALS_RADIUS(2.0);
const float GRIDSIZE(0.5);


int main(int argc, char** argv)
{
	pcl::console::TicToc time;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZ>);

	//load data
	std::cerr << "loading cloud data ... " << std::endl;
	time.tic();
	std::string target_file = "/home/user/data/xian108e/xian2.pcd";
	std::string source_file = "/home/user/data/xian108e/xian5.pcd";

	if (argc > 1)
	{
		target_file = argv[1];
		source_file = argv[2];
	}
	std::cout << target_file << std::endl;
	std::cout << source_file << std::endl;

	pcl::io::loadPCDFile(target_file, *cloud_target);
	pcl::io::loadPCDFile(source_file, *cloud_source);

	std::cout << "target point size:" << cloud_target->points.size() << std::endl;
	std::cout << "source point size:" << cloud_source->points.size() << std::endl;
	std::cout << "[time: " << time.toc() << " ms ]" << std::endl;

	//pro-process
	std::cerr << "cloud data filtering ..." << std::endl;
	time.tic();

	filtered_target = voxelFilter(cloud_target, GRIDSIZE);
	filtered_source = voxelFilter(cloud_source, GRIDSIZE);

	Eigen::Vector4f centroid;  //质心 
	centringXY(filtered_source, filtered_target, centroid);

	std::cout << "fitered target point size:" << filtered_target->points.size() << std::endl;
	std::cout << "fitered target point size:" << filtered_source->points.size() << std::endl;
	std::cout << "[time: " << time.toc() << " ms ]" << std::endl;


	//ground segmentation
	// 模型系数

	std::cerr << "ground segmentating ..." << std::endl;
	time.tic();

	pcl::ModelCoefficients::Ptr coefficients_target(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_target(new pcl::PointIndices);//内点索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_target(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr other_target(new pcl::PointCloud<pcl::PointXYZ>);
	planeSeg(filtered_target, inliers_target, coefficients_target, 1.0);

	pcl::ModelCoefficients::Ptr coefficients_source(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_source(new pcl::PointIndices);//内点索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_source(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr other_source(new pcl::PointCloud<pcl::PointXYZ>);
	planeSeg(filtered_source, inliers_source, coefficients_source, 1.0);


	//compute translate matrix
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f transform_target = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_source = Eigen::Matrix4f::Identity();

	transform_target(2, 3) = coefficients_target->values[3];
	transform_source(2, 3) = coefficients_source->values[3];
	std::cout << transform_target << std::endl;
	std::cout << transform_source << std::endl;

	pcl::transformPointCloud(*filtered_target, *cloud_target_transformed, transform_target);
	pcl::transformPointCloud(*filtered_source, *cloud_source_transformed, transform_source);


	//Eigen::Matrix4f ICP
	Eigen::Matrix4f matrix_icp = ICP(cloud_source_transformed, cloud_target_transformed, 5.0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_align (new pcl::PointCloud<pcl::PointXYZ>);
// 	Eigen::Matrix4f matrix_transform;
// 	matrix_transform = matrix_icp.inverse();
// 	pcl::transformPointCloud(*cloud_source_transformed, *cloud_source_align, matrix_icp);
// 	pcl::io::savePCDFileBinary("d:\\registration\\cloud_source_align.pcd", *cloud_source_align);
// 	pcl::io::savePCDFileBinary("d:\\registration\\cloud_target_transformed.pcd", *cloud_target_transformed);

	//save data
	std::cerr << "save data ..." << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgb_target(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgb_source(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::io::loadPCDFile(target_file, *rgb_target);
	pcl::io::loadPCDFile(source_file, *rgb_source);

	pcl::transformPointCloud(*rgb_target, *rgb_target, transform_target);
	pcl::transformPointCloud(*rgb_source, *rgb_source, matrix_icp * transform_source);

	//pcl::io::savePCDFileBinary("d:\\registration\\rgb_target.pcd", *rgb_target);
	pcl::io::savePCDFileBinary("/home/user/data/xian108e/align.pcd", *rgb_source);

	return (0);
}