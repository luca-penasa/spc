#include <spc/methods/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
//#include <pcl/filters/bilateral_omp.h>
#include <pcl/filters/bilateral.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

double default_sigma_r = 1;
double default_sigma_s = 1;

void
printHelp (int argc, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -ss X = standard deviation of the gaussian kernel to be used for spatial weighing. Default: ");
  print_value ("%f", default_sigma_s); print_info (")\n");

  print_info ("                     -sr X = standard deviation of the gaussian kernel to be used for intensity weighing. Dafault: ");
  print_value ("%f", default_sigma_r); print_info (")\n");
}

bool
loadCloud (const std::string &filename,pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms: "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}



void
compute (const pcl::PCLPointCloud2::ConstPtr &input,pcl::PCLPointCloud2::Ptr &output, const double sigma_r, const double sigma_s)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Bilaterally smoothing the intensity field of the cloud with distance sigma %f, and intensity sigma %f \n", sigma_s, sigma_r);
  //Convert blob to XYZI cloud
  PointCloud<PointXYZI>::Ptr xyzi_cloud (new pcl::PointCloud<PointXYZI> ());
   
  fromPCLPointCloud2 (*input, *xyzi_cloud);
  



  PointCloud<PointXYZI>::Ptr xyzi_cloud_filtered (new PointCloud<PointXYZI> ());
   
  //Create  the filter
  pcl::BilateralFilter<PointXYZI> filter;
 
  //Configure output cloud
//   xyzi_cloud_filtered->points.resize (xyzi_cloud->points.size ());
//   xyzi_cloud_filtered->header = xyzi_cloud->header;
//   xyzi_cloud_filtered->width = xyzi_cloud->width;
//   xyzi_cloud_filtered->height = xyzi_cloud->height;

  
//  pcl::KdTreeFLANN<PointXYZI>::ConstPtr tree (new pcl::KdTreeFLANN<PointXYZI>);

   
   filter.setInputCloud (xyzi_cloud);
//    filter.setSearchMethod (*tree);
   filter.setHalfSize (sigma_s);
   filter.setStdDev (sigma_r);
   filter.filter (*xyzi_cloud_filtered);
   
   
  //set up the filter
  /*filter.setInputCloud(xyzi_cloud);
  filter.setHalfSize(sigma_s);
  filter.setStdDev(sigma_r);*/
  

//   filter.applyFilter();
 
//   boost::mt19937 rng; rng.seed (time (0));
//   boost::normal_distribution<> nd (0, standard_deviation);
//   boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);
// 
//   for (size_t point_i = 0; point_i < xyz_cloud->points.size (); ++point_i)
//   {
//     xyz_cloud_filtered->points[point_i].x = xyz_cloud->points[point_i].x + var_nor ();
//     xyz_cloud_filtered->points[point_i].y = xyz_cloud->points[point_i].y + var_nor ();
//     xyz_cloud_filtered->points[point_i].z = xyz_cloud->points[point_i].z + var_nor ();
//   }

//  pcl::PCLPointCloud2 output;
  pcl::toPCLPointCloud2 (*xyzi_cloud_filtered, *output);
//    concatenateFields (*input, *xyzi_cloud_filtered, output); TODO add this for keep the fields

//   print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms: "); print_value ("%d", output.width * output.height); print_info (" points]\n");
 }

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms: "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Smooth the intensity field of the cloud with a bilateral filter, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  double sigma_s = default_sigma_r;
  double sigma_r = default_sigma_s;
  parse_argument (argc, argv, "-ss", sigma_s);
  parse_argument (argc, argv, "-sr", sigma_r);

  // Load the first file
 pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);
  
  // check if the intensity field exist
    if (getFieldIndex(*cloud, "intensity") == -1)
    {
	    print_error ("Input cloud must have an intensity field.\n");
	    return (-1);
    }

  // Do the filtering
 pcl::PCLPointCloud2::Ptr output (new pcl::PCLPointCloud2);
  
  
  compute (cloud, output, sigma_r, sigma_s);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], *output);
}
