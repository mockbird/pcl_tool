#include <iostream>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include <Eigen/Dense>

#include <vtkPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkCellData.h>
#include <vtkTriangle.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/timer.hpp>

inline double uniform_deviate (int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

inline void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector3f& p)
{
    float r1 = static_cast<float> (uniform_deviate (rand ()));
    float r2 = static_cast<float> (uniform_deviate (rand ()));
    float r1sqr = std::sqrt (r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
}

void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize){
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setLeafSize(gridsize, gridsize, gridsize);
  vox_grid.setInputCloud(cloud_in);
  vox_grid.filter(*cloud_out);
  return;
}

void saveMeshModel(std::vector< std::vector<int> > meshIdIndex,vtkPolyData* vtkPolyDataBim, const char* pathName)
{
  vtkSmartPointer<vtkPoints> points =  vtkSmartPointer<vtkPoints>::New();
    points = vtkPolyDataBim->GetPoints();
    std::cout << points->GetNumberOfPoints() << std::endl;
    
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();

    for(int i=0;i<meshIdIndex.size();i++)  
    {
	vtkSmartPointer<vtkTriangle> trianle = vtkSmartPointer<vtkTriangle>::New();
	trianle->GetPointIds()->SetId(0, meshIdIndex[i][0]);
	trianle->GetPointIds()->SetId(1, meshIdIndex[i][1]);
	trianle->GetPointIds()->SetId(2, meshIdIndex[i][2]);
	cells->InsertNextCell(trianle);
    }
    vtkSmartPointer<vtkPolyData> polygonPolyData =  vtkSmartPointer<vtkPolyData>::New();
    polygonPolyData->SetPoints(points);
    polygonPolyData->SetPolys(cells);

    vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter->SetFileName(pathName);
    plyWriter->SetInput(polygonPolyData);
    plyWriter->Write();
}

int main(int argc, char **argv)
{
    double meshReslotion = 0.3;
    float gridSize = 0.1;
    //read point-cloud file
     pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointcloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/user/data/schedule/cloud_align.ply", *pclPointcloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
    std::cout << "point num :" << pclPointcloud->points.size() << std::endl;
    
    voxelFilter(pclPointcloud,pclPointcloud,gridSize);
    std::cout << "point num :" << pclPointcloud->points.size() << std::endl;

    boost::timer tree; //start time
  
    pcl::KdTreeFLANN<pcl::PointXYZ> pclKdtree;
  //   pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    pclKdtree.setInputCloud (pclPointcloud);

    std::cout<<"运行时间："<<tree.elapsed() <<"s"<<std::endl;//输出已流失的时间
  
    pcl::PointXYZ searchPoint;
    
    int K =1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
  
    //read cad file, fill data into polydata
    
    vtkSmartPointer<vtkPLYReader> plyReader = vtkSmartPointer<vtkPLYReader>::New();
    plyReader->SetFileName("/home/user/data/schedule/BIM.ply");
  
    vtkSmartPointer<vtkPolyData> vtkPolyDataBim = vtkSmartPointer<vtkPolyData>::New();
    vtkPolyDataBim = plyReader->GetOutput();
    vtkPolyDataBim->UpdateData();

    
    std::vector<std::vector<int> > meshIdIndexCompeleted;
    std::vector<std::vector<int> > meshIdIndexOnGoing;
      
    Eigen::Vector3f randomPointInTriange;
    for(vtkIdType i = 0; i < vtkPolyDataBim->GetNumberOfCells(); i++)
    {
	vtkCell* cell = vtkPolyDataBim->GetCell(i);
      
	vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
	
	double p1[3],p2[3],p3[3];
	triangle->GetPoints()->GetPoint(0, p1);
	triangle->GetPoints()->GetPoint(1, p2);
	triangle->GetPoints()->GetPoint(2, p3);
	double vtkTringleArea = vtkTriangle::TriangleArea(p1,p2,p3);
	int samplePointsNum = ceil(vtkTringleArea/meshReslotion);
	int activateCount = 0;
	for (int j=0;j<samplePointsNum;j++)
	{

	  randomPointTriangle(p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],p3[0],p3[1],p3[2],randomPointInTriange);
	  searchPoint.x = randomPointInTriange[0];
	  searchPoint.y = randomPointInTriange[1];
	  searchPoint.z = randomPointInTriange[2];
	  
	  pclKdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) ;
	  if (pointNKNSquaredDistance[0]<0.5)
	    activateCount++;
	}
	float score = float(activateCount)/float(samplePointsNum);
	
	std::vector<int> triangleVertexId;
	triangleVertexId.push_back(triangle->GetPointId(0));
	triangleVertexId.push_back(triangle->GetPointId(1));
	triangleVertexId.push_back(triangle->GetPointId(2));
	  
	if (score > 0.5)
	{
	  meshIdIndexCompeleted.push_back(triangleVertexId);
	}
	else
	{
	  meshIdIndexOnGoing.push_back(triangleVertexId);
	}
// 	std::cout <<  "No:" <<  i << " score:" << score << "," <<  activateCount << "," << samplePointsNum <<  std::endl;
    }
    saveMeshModel(meshIdIndexCompeleted,vtkPolyDataBim,"compeleted.ply");
    saveMeshModel(meshIdIndexOnGoing,vtkPolyDataBim,"on-going.ply");

    return 0;
}

