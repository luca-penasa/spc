#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls_omp.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

double search_radius = 0.05;
double sqr_gauss_param = search_radius*search_radius;
bool use_polynomial_fit = true;
int polynomial_order = 2;


int
  main (int argc, char *argv[])
{
  std::string infilename = argv[1];
  std::string outfilename = argv[2];
//  float search_radius = atof(argv[3]);
  // Load input file into a PointCloud<T> with an appropriate type
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
  sensor_msgs::PointCloud2 cloud_blob;
  loadPCDFile (infilename.c_str(), cloud_blob);
  fromROSMsg (cloud_blob, *cloud);


  // Init object (second point type is for the normals, even if unused)
  MovingLeastSquaresOMP<PointXYZ, PointNormal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  PointCloud<PointNormal>::Ptr out_cloud (new PointCloud<PointNormal> ());


  // Set parameters.
  mls.setNumberOfThreads(2);

          mls.setInputCloud (cloud);
          mls.setSearchRadius (search_radius);
          mls.setSqrGaussParam (sqr_gauss_param);
          mls.setPolynomialFit (use_polynomial_fit);
          mls.setPolynomialOrder (polynomial_order);

        //  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
        //  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
       //   mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);
//          mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::NONE);
//         mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
//          mls.setUpsamplingRadius (0.025);
//          mls.setUpsamplingStepSize (0.015);
//          mls.setDilationIterations (2);
//          mls.setDilationVoxelSize (0.01f);

          search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
          mls.setSearchMethod (tree);
          mls.setComputeNormals (true);



          mls.process (*out_cloud);


//sensor_msgs::PointCloud2 out_blob;
//toROSMsg(*out_cloud, out_blob);

  // Save output
  savePCDFileBinary (outfilename.c_str(), *out_cloud);

}
