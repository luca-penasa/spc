#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/impl/io.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/features/impl/rift.hpp>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/impl/intensity_spin.hpp>

//0 -> intensity_gradient RIFTE
//1 -> intensity_spin 
const int feature_type = 0;

const int nr_intensity_bins = 3; //if using SPIN
const int nr_gradient_bins = 3;  //if using gradient
const int nr_distance_bins = 3;  //used both with spin and gradient

const int bins_number = 9;





typedef pcl::PointXYZI PointT;
typedef pcl::Normal NormalT;
typedef pcl::IntensityGradient GradientT;
typedef pcl::Histogram<bins_number> HistoT;

POINT_CLOUD_REGISTER_POINT_STRUCT(HistoT,
					(float[bins_number], histogram, histogram)
	
)



int main(int argc, char *argv[])
{
	


	std::string infilename = argv[1];
	//std::string outfilename = argv[2];
	float radius = atof(argv[2]);
	
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(infilename.c_str(), *cloud);
	pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
	
	if (feature_type == 0)
	{
			// Create the normal estimation class, and pass the input dataset to it
			pcl::NormalEstimation<PointT, NormalT> ne;
			ne.setInputCloud (cloud);

			// Create an empty kdtree representation, and pass it to the normal estimation object.
			// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			ne.setSearchMethod (tree);

			// Output datasets
			pcl::PointCloud<NormalT>::Ptr cloud_normals (new pcl::PointCloud<NormalT>);

			// Use all neighbors in a sphere of radius 3cm
			ne.setRadiusSearch (radius);

			// Compute the features
			printf("Computing Normals\n");
			ne.compute (*cloud_normals);
			//pcl::copyPointCloud(*cloud, *cloud_normals);
			
			//Now compute the intensity gradient
			pcl::PointCloud<GradientT>::Ptr cloud_gradient (new pcl::PointCloud<GradientT>);
			
			pcl::IntensityGradientEstimation <PointT, NormalT, GradientT> gr;
			gr.setInputCloud (cloud);
			gr.setInputNormals (cloud_normals);
			gr.setSearchMethod (tree);
			gr.setRadiusSearch (radius);
			printf("Computing Gradient\n");
			gr.compute(*cloud_gradient);
			
		// 	pcl::PointCloud<PointFullT>::Ptr cloud_full (new pcl::PointCloud<PointFullT>);
		// 	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_full);
		// 	pcl::concatenateFields(*cloud_full, *cloud_gradient, *cloud_full);
			
			//Estimate RIFTE
			printf("Computing RIFTE Descriptors.\n");
			pcl::PointCloud<HistoT>::Ptr cloud_rift (new pcl::PointCloud<HistoT>);
			
			pcl::RIFTEstimation< PointT, GradientT, HistoT > rift;
			rift.setInputCloud(cloud);
			rift.setInputGradient(cloud_gradient);
			rift.setNrDistanceBins(nr_distance_bins);
			rift.setNrGradientBins(nr_gradient_bins);
			rift.setSearchMethod(tree);
			rift.setRadiusSearch(radius);
			rift.compute(*cloud_rift);
			pcl::io::savePCDFile("RIFTE.pcd", *cloud_rift);
	}
		
	else if (feature_type == 1)
	{
			printf("Computing Spin descriptors.\n");
			pcl::PointCloud<HistoT>::Ptr cloud_spin (new pcl::PointCloud<HistoT>);
			pcl::IntensitySpinEstimation< PointT, HistoT > spi;
			spi.setInputCloud(cloud);
			spi.setNrIntensityBins(nr_intensity_bins);
			spi.setNrDistanceBins(nr_distance_bins);
			spi.setSearchMethod(tree);
			spi.setRadiusSearch(radius);
			spi.compute(*cloud_spin);
			pcl::io::savePCDFile("SPIN.pcd", *cloud_spin);
			
	}
	//pcl::io::savePCDFile("gradient.pcd", *cloud_gradient);
}
