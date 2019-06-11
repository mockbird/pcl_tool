#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k)
{
 double res = 0.0;
 int n_points = 0;
 pcl::KdTreeFLANN<pcl::PointXYZ> tree;
 tree.setInputCloud(cloud);
 //?what means size_t
 for (size_t i = 0; i < cloud->size(); ++i)
 {
  if (!pcl_isfinite((*cloud)[i].x))
   //pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
  {
   continue;
  }
  std::vector<int> indices(k);
  //创建一个动态数组，存储查询点近邻索引
  std::vector<float> sqr_distances(k);
  //存储近邻点对应平方距离
  if (tree.nearestKSearch(i, k, indices, sqr_distances) == k)
  {
   for (int i = 1; i < k; i++)
   {
    res += sqrt(sqr_distances[i]);
    ++n_points;
   }
  }
 }
 if (n_points != 0)
 {
  res /= n_points;
 }
 return res;
}
int main(int argc, char** argv)
{
 //创建一个点云
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 cloud->width = 1000;
 cloud->height = 1;
 cloud->points.resize(cloud->width* cloud->height);
 
 for (size_t i = 0; i < cloud->points.size(); ++i)
 {
  cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
  //std::cout << cloud->points[i].x;
  cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
  //std::cout << cloud->points[i].y;
  cloud->points[i].z= 1024.0f*rand() / (RAND_MAX + 1.0f);
  //std::cout << cloud->points[i].z;
 }
 float des = computeCloudResolution(cloud, 4);
 std::cout << des;
 system("pause");
 return 0;
}