#pragma once

#include <iostream>

//filter
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>//按索引提取点云
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

//seg 
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//visualization
//#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>

#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d_omp.h>

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

/** \brief 体素格采样，使点云密度均匀，压缩点云数量
* \param[in] 源点云
* \param[in] 体素格边长，体素格为正方形
* \param[out] 采样后点云
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float gridsize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setLeafSize(gridsize, gridsize, gridsize);
	vox_grid.setInputCloud(cloud_in);
	vox_grid.filter(*cloud_out);
	return cloud_out;
}

/** \brief Statistical Outlier Removal离群点剔除
* \param[in] 源点云
* \param[in] 邻域点个数
* \param[in] the standard deviation multiplier for the distance threshold calculation.
* \param[out] 去除离群点后的点云
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr SOR(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
	float meank = 20,
	float StddevMulThresh = 0.5)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setMeanK(meank);
	sor.setStddevMulThresh(StddevMulThresh);
	sor.filter(*cloud_out);
	std::cout << "SOR cooridate:" << std::endl;
	std::cout << "meank=" << meank << ", Stddev=" << StddevMulThresh << std::endl;
	return cloud_out;
}

/** \brief RANSAC平面识别
* \param[in]
* \param[out]
*/
void planeSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
	pcl::PointIndices::Ptr inliers,
	pcl::ModelCoefficientsPtr &coefficients,
	float DistanceThreshold)
{
	//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//内点索引
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);//　平面模型
	seg.setMethodType(pcl::SAC_RANSAC);// 随机采样一致性算法
	Eigen::Vector3f v(0, 0, 1);
	seg.setAxis(v);
	seg.setEpsAngle(5 / 180 * 3.14);

	seg.setInputCloud(cloud_in);//输入点云
	seg.setMaxIterations(200);
	seg.setDistanceThreshold(DistanceThreshold);//是否在平面上的阈值
	seg.segment(*inliers, *coefficients);//分割　得到平面系数　已经在平面上的点的　索引

										 //     if (inliers->indices.size () == 0)
										 //     {
										 //       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
										 //       return (-1);
										 //     }
										 // 打印平面系数
	std::cerr << "Model Plane coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
}

/** \brief 地面投影，将三维点云按指定投影方向投影，得到二维点云
* \param[in]
* \param[out]
*/
void groundProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
	pcl::ModelCoefficientsPtr &coefficients)
{
	pcl::ProjectInliers<pcl::PointXYZ> proj;

	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_in);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_out);
	return;
}

/** \brief 点云法方向计算
* \param[in]
* \param[out]
*/
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, double normals_radius) {
	pcl::PointCloud<pcl::Normal>::Ptr normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud(incloud);
	norm_est.setRadiusSearch(normals_radius);
	norm_est.compute(*normalsPtr);
	return normalsPtr;
}

/** \brief 点云凹包提取，使用了Alphashape
* \param[in]
* \param[out]
*/
void hull(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)

{
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_in);
	chull.setAlpha(1);
	chull.reconstruct(*cloud_out);
	std::cout << cloud_in->points.size() << std::endl;
	std::cout << "hull cloud size :" << cloud_out->points.size() << std::endl;
}

/** \brief 垂直面提取
* \param[in]
* \param[out]
*/
void verticalExtractor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)

{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = getNormals(cloud_in, 2.0);
	Eigen::Vector3d a(0, 0, 1);
	for (int i = 0; i<cloud_normals->points.size(); i++)
	{
		Eigen::Vector3d n(cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z);
		double cosa = a.dot(n) / a.norm() / n.norm();
		double angel = acos(cosa) / 3.14159265 * 180;

		pcl::PointXYZ p;
		p.x = cloud_in->points[i].x;
		p.y = cloud_in->points[i].y;
		p.z = cloud_in->points[i].z;

		if (abs(angel - 90)<15)
		{
			cloud_out->push_back(p);
		}
	}
	return;
}

/** \brief 在XY平面上将点云和BIM中心化
* \param[in]
* \param[out]
*/
void centringXY(pcl::PointCloud<pcl::PointXYZ>::Ptr &bim,
				pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
				Eigen::Vector4f &centroid)
{

	pcl::compute3DCentroid(*bim, centroid); //估计质心的坐标
	Eigen::Matrix4f centroid_matrix = Eigen::Matrix4f::Identity();
	//     centroid_matrix.block<4,1>(0,3) = -centroid;
	centroid_matrix(0, 3) = -centroid[0];
	centroid_matrix(1, 3) = -centroid[1];
	pcl::transformPointCloud(*cloud, *cloud, centroid_matrix);
	pcl::transformPointCloud(*bim, *bim, centroid_matrix);
	return;
}

/** \brief 在XY平面上裁切点云
* \param[in] width 切边宽度
* \param[out]
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCutXY(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, Eigen::Vector4f minPt, Eigen::Vector4f maxPt, float width = 20.0)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::getMinMax3D(*cloud_in,minPt,maxPt);

	//std::cout << "minPt" << minPt << std::endl;
	//std::cout << "maxPt" << maxPt << std::endl;

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(minPt[0] - width, maxPt[0] + width);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_out);

	pass.setInputCloud(cloud_out);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(minPt[1] - width, maxPt[1] + width);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_out);

	return cloud_out;
}

Eigen::Matrix4f ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
	float CorrespondenceDistance = 10.0,
	int MAX_ICP_ITERATION = 300)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaximumIterations(MAX_ICP_ITERATION);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setTransformationEpsilon(1e-10);
	icp.setMaxCorrespondenceDistance(CorrespondenceDistance);
	pcl::PointCloud<pcl::PointXYZ> Final;         //存储经过配准变换点云后的点云
	icp.align(Final);
	Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
	return icp_transform;
}


//void viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al) {
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->initCameraParameters();
//
//	int v1(0), v2(0);
//	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
//	//   viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
//	//   viewer->initCameraParameters();   //初始化相机参数
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud1, 0, 255, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);
//	viewer->addPointCloud(cloud1, green, "v1_target", v1);
//	viewer->addPointCloud(cloud2, red, "v1_sourse", v1);
//
//
//	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
//	viewer->addText("After Alignment", 10, 10, "v2 text", v2);
//	//   viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green2(cloud1al, 0, 255, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red2(cloud2al, 255, 0, 0);
//	viewer->addPointCloud(cloud1al, green2, "v2_target", v2);
//	viewer->addPointCloud(cloud2al, red2, "v2_sourse", v2);
//	viewer->spin();
//
//	return;
//}