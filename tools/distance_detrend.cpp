#include <spc/common/std_helpers.hpp>
#include <spc/methods/kernel_smoothing.cpp>
#include <pcl/io/pcd_io.h>
#include <spc/methods/nn_interpolator.h>
#include <spc/common/common.h>

//#include <spc/methods/linear_interpolator.h>

#include <pcl/filters/voxel_grid.h>

#include <spc/common/io_helper.h>

#include <spc/common/strings.h>
#include <spc/common/point_types.h>

#include <boost/lexical_cast.hpp>

#include <spc/methods/kernelsmoothing2.h>


void printHelp()
{
    std::cout << "USAGE: distance_detrend filename.pcd bandwidth voxelsize" << std::endl;
}

int main(int argc, char *argv[])
{

    printHelp();
    float bandwidth = atof(argv[2]);
    float voxelsize = atof(argv[3]);

    std::string bandwidth_string = std::string("b_") + boost::lexical_cast<std::string>(bandwidth);
    std::string voxelsize_string = std::string("v_") + boost::lexical_cast<std::string>(voxelsize);


	//input cloud
    std::string infilename = argv[1];
    std::string basename = spc::stripExtension(infilename); //the filename without extension

	//load the cloud
	pcl::PCLPointCloud2::Ptr cloud  (new pcl::PCLPointCloud2);
	pcl::io::loadPCDFile(infilename, *cloud);

    pcl::PointCloud<PointIntDist> cloud_;
	pcl::PointCloud<pcl::PointXYZ> cloud_xyz_;

	//get distance and intensity fields
	fromPCLPointCloud2(*cloud, cloud_);

    //also the xyz cloud
	fromPCLPointCloud2(*cloud, cloud_xyz_);

    //downsample xyz cloud, to speed up the trend estimation
    //using voxelgrid to obtain an even sampling of the cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> grid_filter;
    grid_filter.setInputCloud(cloud);
    grid_filter.setLeafSize(voxelsize, voxelsize, voxelsize); //5 cm x 5 x 5 as boxsize for downsampling

   pcl::PCLPointCloud2::Ptr downsampled_cloud  (new pcl::PCLPointCloud2);
    grid_filter.filter(*downsampled_cloud);

    //keep a copy of the downsampled cloud for future inspection
    pcl::io::savePCDFile(basename + voxelsize_string + bandwidth_string + std::string("_downsampled.pcd"), *downsampled_cloud);

    std::cout << " Downsampled version saved  " << std::endl;

    pcl::PointCloud<PointIntDist> cloud_down; //a cloud with only d and I, for the downsampled version
    fromPCLPointCloud2(*downsampled_cloud, cloud_down);


	std::vector<float> i_vector;
	std::vector<float> d_vector;



    int n = cloud_down.size();
	i_vector.resize(n);
	d_vector.resize(n);

	//vectorialize the cloud_
	for (int i = 0; i < n; ++i)
	{
        i_vector[i] = cloud_down[i].intensity;
        d_vector[i] = cloud_down[i].distance;
    } //we here have i and d for all the points in the downsampled cloud


    float min = spc::get_min(d_vector);
    float max = spc::get_max(d_vector);
    float step = 0.04; //every 4 cm get an estimate

    //the vector with distances at which to estimate the trend
    std::vector<float> new_d = spc::subdivideRange(min, max, step);
    spc::EquallySpacedTimeSeries<float>::Ptr  series;

    //now do the estimate of the trend, using kernelsmoothing
	//now initialize a kernelsmoothing object
    spc::KernelSmoothing2<float> ks;
//    ks.setComputeVariance(1); //also compute the variance

    ks.setInputSeries(spc::SparseTimeSeries<float>::Ptr(new spc::SparseTimeSeries<float>(d_vector, i_vector)));
//	ks.setXY(d_vector, i_vector);
//	ks.setEvaluationPositions(new_d);
    ks.setBandwidth(bandwidth); //using a bandwidth of 1 meters



    //just some min/max info
    std::cout << "min distance: " << min << "  max distance: " << max << std::endl;


    std::cout << "Computing Kernel smoothing model on subsampled cloud." << std::endl;

	//now compute ks
    ks.compute();

    series = ks.getOutputSeries();

    //get the computed values
    std::vector<float> new_i = series->getY();
//	std::vector<float> vars = ks.getVar();

    //save some data for any future use
    std::vector< std::vector<float> > tmp_out1;
    tmp_out1.push_back(new_d);
    tmp_out1.push_back(new_i);

    std::cout << "Saving the correction curve" << std::endl;
    spc::saveAsCSV(basename + voxelsize_string + bandwidth_string + std::string("_correction_curve.txt") , std::string(" "), tmp_out1);

    spc::fill_nan_rbf<float>(new_d, new_i ); //eventually fill-in nans using RBF interpolator




    //NOW WE NEED TO COMPUTE A TREND VALUE FOR EACH POINT OF THE UN-DOWNSAMPLED CLOUD
    //THIS WOULD BE TOO TIME-CONSUMING FOR USING RBF AS ABOVE.
    //SO AN ALTERNATIVE AND FASTER INTERPOLATOR IS GOING TO BE USED

    std::cout << "Starting with the fast interpolation" << std::endl;

	//put up a nnInterpolator class TODO finish to change to rbf interpolator


    spc::LinearInterpolator<float> interpolator;

    float mymin =  spc::get_min(new_d);
    float mystep =  step;



    interpolator.setXY(new_i, mystep, mymin);

    std::vector<float> full_d_vector;
    full_d_vector.reserve(cloud_.size());
    full_d_vector.resize(cloud_.size());

    for (int i = 0; i < cloud_.size(); ++i)
    {
        full_d_vector.at(i) =  cloud_.at(i).distance; //simple copy of data
    }

    interpolator.setNewX(full_d_vector);
    //interpolator.updateInternalTables();

    interpolator.compute();

    std::vector<float> output = interpolator.getNewY();



    for (int i = 0; i < cloud_.size(); ++i)
    {
        cloud_[i].intensity = cloud_[i].intensity / output[i];

    }

       pcl::PCLPointCloud2 new_i_cloud;
        pcl::toPCLPointCloud2(cloud_, new_i_cloud);

       pcl::PCLPointCloud2 out_cloud;
        pcl::PointCloud<pcl::PointXYZI> definitive_cloud;

        pcl::concatenateFields(*cloud,new_i_cloud, out_cloud);

        pcl::fromPCLPointCloud2(out_cloud, definitive_cloud);

        pcl::io::savePCDFileBinary(basename + voxelsize_string + bandwidth_string + std::string("_corrected_ints.pcd"), definitive_cloud);

}
