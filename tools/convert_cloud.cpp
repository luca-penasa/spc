#include <vtkParticleReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPointData.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkPointSet.h>
#include <vtkUnstructuredGridReader.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <sensor_msgs/PointCloud2.h>


#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void vtkUnstructuredGrid2PointCloud2 (vtkSmartPointer<vtkUnstructuredGrid> in_cloud, sensor_msgs::PointCloud2::Ptr out_cloud)
{
	int n_points = in_cloud->GetNumberOfPoints();
	print_info("n points: %i\n", n_points);
}

	

void writePointCloud2(std::string file_name, sensor_msgs::PointCloud2::Ptr cloud)
{
	savePCDFile(file_name, *cloud);
	
}



int 
main (int argc, char* argv[])
{
	
	//Setting up the reader
	vtkSmartPointer<vtkUnstructuredGridReader> reader = vtkSmartPointer<vtkUnstructuredGridReader>::New();
	reader->SetFileName(argv[1]);
	reader->Update();
	
	//Getting output from reader
	vtkSmartPointer<vtkUnstructuredGrid> cloud = vtkSmartPointer<vtkUnstructuredGrid>::New();
	cloud = reader->GetOutput();

	
	//Creating final PointCloud2
	sensor_msgs::PointCloud2::Ptr new_cloud  (new sensor_msgs::PointCloud2);
	vtkUnstructuredGrid2PointCloud2(cloud, new_cloud);
	
	//Write pointcloud
	writePointCloud2("writed_cloud.pcd", new_cloud);
	
	return(1);
}


