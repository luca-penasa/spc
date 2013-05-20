#include <spc/common/std_helpers.hpp>

#include <pcl/io/pcd_io.h>
#include <spc/methods/nn_interpolator.h>
#include <spc/common/common.h>

#include <pcl/console/parse.h>

#include <spc/common/point_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/io/io.h>

#include <pcl/filters/voxel_grid.h>

int main(int argc, char *argv[])
{

    float plane_normal[3] = {1,0,0};
    //input cloud
    std::string infilename = argv[1];
    float grid_size = atof(argv[2]);


    //load the cloud
    sensor_msgs::PointCloud2::Ptr cloud  (new sensor_msgs::PointCloud2);


    pcl::io::loadPCDFile(infilename, *cloud);



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected ( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz ( new pcl::PointCloud<pcl::PointXYZ>);

    fromROSMsg(*cloud, *cloud_xyz);

    //so project cloud on plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = plane_normal[0];
    coefficients->values[1] = plane_normal[1];
    coefficients->values[2] = plane_normal[2];

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_xyz);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);



    sensor_msgs::PointCloud2::Ptr new_cloud  (new sensor_msgs::PointCloud2);
    toROSMsg(*cloud_projected, *new_cloud);

    sensor_msgs::PointCloud2::Ptr new_cloud2  (new sensor_msgs::PointCloud2);

    pcl::concatenateFields(*cloud, *new_cloud, *new_cloud2);



    pcl::io::savePCDFile("projected.pcd", *new_cloud2);







    //now downsample with a grig
    pcl::PointCloud<pcl::PointXYZ>::Ptr gridded_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vgrid_filter;
    vgrid_filter.setInputCloud(cloud_projected);
    vgrid_filter.setLeafSize(grid_size, grid_size, grid_size);
    vgrid_filter.filter(*gridded_cloud);


    pcl::io::savePCDFileBinary("gridded.pcd", *gridded_cloud);









    //2d cloud




    return (-1);
}
