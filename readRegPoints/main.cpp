#include <iostream>
#include <fstream>
#include <ostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

void readRegpoints(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr sourceRegPoints,
				    pcl::PointCloud<pcl::PointXYZ>::Ptr targetRegPoints
 				  )
{
  std::vector <double> RegPointSet;
  std::ifstream infile; 
  infile.open(filename);  
  
  char c;
  std::string strValue, strPoint;
  std::vector <std::string> vecStr;
  
  while (!infile.eof())
  {
    infile >> c;
    if (c == '[')
      strValue.clear();
    else if (c == ']' )
    {
      strValue.push_back(',');
      vecStr.push_back(strValue);
    }
    else if (c != '\"')
      strValue.push_back(c);
  }
  
  for (size_t i=0;i<vecStr.size();i++)
  {

    for (size_t j=0;j<vecStr[i].size();j++)
    {
      if (vecStr[i][j] ==',')
      {
	RegPointSet.push_back(std::strtod(strPoint.c_str(),NULL));
	strPoint.clear();
      }
      else
	strPoint.push_back(vecStr[i][j]);
    }
  }
  
  for (int i=0;i<RegPointSet.size()/6;i++)
  {
    pcl::PointXYZ p;
    p.x = RegPointSet[i];
    p.y = RegPointSet[i+1];
    p.z = RegPointSet[i+2];
    sourceRegPoints->points.push_back(p);
    p.x = RegPointSet[i+3];
    p.y = RegPointSet[i+4];
    p.z = RegPointSet[i+5];
    targetRegPoints->points.push_back(p);

  }

}
int main(int argc, char **argv) {
  
  std::cout << "hello" << std::endl;
  std::string filename = "/home/user/ts.txt";

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceRegPoints(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetRegPoints(new pcl::PointCloud<pcl::PointXYZ>());
  readRegpoints(filename, sourceRegPoints, targetRegPoints);

  pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ, pcl::PointXYZ, float> TESVD;
  pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 transformation;

  std::cout << "transform matrix is:" << std::endl;
  std::cout << transformation << std::endl;
  std::cout << transformation(0,0) << std::endl;

  std::ofstream matrix("matrix.txt");
  for (int i = 0; i<4; i++)
  {
      matrix << transformation(i,0) << "," << transformation(i,1) << "," << transformation(i,2) << "," << transformation(i,3)<< std::endl;
  }
//   cout << "trans has done!!!" << endl;
  std::cin.get();


    return 0;
}
