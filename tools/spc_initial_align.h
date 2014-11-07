#include <pcl/console/print.h> //for printing
#include <pcl/console/parse.h> //for parsing
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/features/normal_3d.h"
#include <pcl/features/normal_3d_omp.h>
// #include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/keypoints/sift_keypoint.h> //for sift keypoints

#include <pcl/v

#include "pcl/features/fpfh.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>

#include "pcl/registration/ia_ransac.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/correspondence.h>


#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/keypoints/impl/harris_keypoint3D.hpp>


#include <numeric> // for accumulate function
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;

// typedef pcl::PointXYZI PointInT;
// typedef pcl::PointXYZI PointOutT;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;


typedef pcl::PointXYZI PointIntT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZ KeysT;


using namespace pcl::console;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct sac_ia_results{
	double fitness_score;
	Eigen::Matrix4f final_transformation;
	bool has_converged;
	
	void 
	print()
	{
		string conv;
		if (has_converged == true)
		{
			conv = "yes";
		}
		else
		{
			conv = "no";
		}
		print_highlight("SAC-IA results:\n");
		print_info("     - Fitness score: \t%4.3f\n", fitness_score);
		print_info("     - Has converged: \t%s\n", conv.c_str());
		print_info("     - Transformation Matrix:\n");
		
		
		Eigen::Matrix3f rotation = final_transformation.block<3,3>(0, 0);
		Eigen::Vector3f translation = final_transformation.block<3,1>(0, 3);
		
		printf ("\n");
		printf ("           | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("       R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("           | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf ("\n");
		printf ("       t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

        printf ("CloudCompare compatible output:\n");
        printf ("%6.8f %6.8f %6.8f %6.8f\n", rotation (0,0), rotation (0,1), rotation (0,2), translation (0));
        printf ("%6.8f %6.8f %6.8f %6.8f\n", rotation (1,0), rotation (1,1), rotation (1,2), translation (1));
        printf ("%6.8f %6.8f %6.8f %6.8f\n", rotation (2,0), rotation (2,1), rotation (2,2), translation (2));
        printf ("%6.8f %6.8f %6.8f %6.8f\n", 0.0, 0.0, 0.0, 1.0);

	}
	
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * STRUCTURES FOR STORING SETTINGS
 * all settings will converge in a unique "all_settings" structure.
 * so we can pass this structure when needed.
 */
struct sift_settings {
	double min_contrast;
	double min_scale;
	int nr_octaves;
	int nr_scales_per_octave;
	
	sift_settings(): min_contrast(100), min_scale(0.03), nr_octaves(20), nr_scales_per_octave(5) {}
	
	int
	parse(int argc, char * argv[])
	{
		parse_argument(argc, argv, "-s-mc", min_contrast);
		parse_argument(argc, argv, "-s-ms", min_scale);
		parse_argument(argc, argv, "-s-no", nr_octaves);
		parse_argument(argc, argv, "-s-nspo", nr_scales_per_octave);
		return 1;
	};
	
	int 
	print() const
	{
		print_info("\n");
		print_highlight("SIFT parameters:\n");
		print_info("  Minimum contrast:                   %.2f\n", min_contrast);
		print_info("  Minimum scale:                      %.2f\n", min_scale);
		print_info("  Number of octaves:                  %i\n", nr_octaves);
		print_info("  Number of scales per octave:        %i\n", nr_scales_per_octave);
		return 1;
	};
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct harris_settings
{
	double radius_search;
	bool refine;
	double threshold;
	
	harris_settings(): radius_search(0.05), refine(true), threshold(10) {}
	int
	parse(int argc, char * argv[])
	{
		parse_argument(argc, argv, "-h-rs", this->radius_search);
		parse_argument(argc, argv, "-h-t", this->threshold);
		string refine;
		int ref_found = parse_argument(argc, argv, "-h-r", refine);
		if (ref_found == 1)
		{
			if (refine == "1" or refine == "yes" or refine =="true" or refine == "y")
			{
				this->refine = true;
			}
			else if (refine == "0" or refine =="no" or refine =="false" or refine == "n")
			{
				this->refine = false;
			}
			else
			{
				print_error("-h-r argument not managed, supply 0/1, yes/no, true/false or y/n please.\n");
				return -1;
			}
		}
		return 1;
	};
	
	void
	print() const
	{
		print_info("\n");
		string is_refine_actived;
		refine ? is_refine_actived = "yes": is_refine_actived= "no";
		print_highlight("Harris parameters:\n");
		print_info("  Radius size:                        %.2f\n", radius_search);
		print_info("  Refine:                             %s\n", is_refine_actived.c_str());
		print_info("  Threshold:                          %.2f\n", threshold);
	};
	
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct normals_settings
{
	int k_search;
	normals_settings(): k_search(10) {}
	
	int 
	parse(int argc, char * argv[])
	{
		parse_argument(argc, argv, "-n-ks", k_search);
		return 1;
	};
	
	void
	print() const
	{
		print_info("\n");
		print_highlight("Normals computation parameters:\n");
		print_info("  Knn size:                        %i\n", k_search);
	};
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct FPFHE_settings
{
	double radius_search;
	int k_search; 
	string search_method;
	FPFHE_settings() : radius_search(0.5), k_search(40), search_method("rs") {}
	
	int 
	parse(int argc, char * argv[])
	{
		int is_radius = parse_argument(argc, argv, "-f-rs", this->radius_search);
		int is_k = parse_argument(argc, argv, "-f-ks", this->k_search);
		
		if (is_radius != -1 && is_k != -1)
		{
			print_error("You MUST chose between radius search and k-nn search for FPFHE computation.\n");
			print_error("You specified both! Just chose between -f-rs and -f-ks flags.\n");
			print_error("Default will be used: ");
			if (this->search_method == "rs")
			{
				print_error("   radius search ");
			}
			else if (this->search_method == "ks")
			{
				print_error("   knn search ");
			}
		}
		else if (is_radius != -1)
		{
			//so use radius search
			this->search_method = "rs";
		}
		else if (is_k != -1)
		{
			this->search_method = "ks";
		}
		return 1;
		
	};
	
	void
	print() const
	{
		print_info("\n");
		print_highlight("FPFHE parameters:\n");
		if (search_method == "rs")
		{
			print_warn("     Radius search will be used. use -f-ks value for knn search.\n");
			print_info("  Radius size:                        %.2f\n", radius_search);
		}
		else if (search_method == "ks")
		{
			print_warn("     Knn search will be used. use -f-rs value for radius search.\n");
			print_info("  K-neighborhood size:              %i\n", k_search);
		}
	};
	
};

struct sac_ia_settings
{
	double min_sample_distance;
	double max_correspondence_distance;
	int nr_iterations;
	sac_ia_settings() : min_sample_distance(0.05), max_correspondence_distance(0.01), nr_iterations(500) {}
	
	int 
	parse(int argc, char * argv[])
	{
		parse_argument(argc, argv, "-si-msd", this->min_sample_distance);
		parse_argument(argc, argv, "-si-mcd", this->max_correspondence_distance);
		parse_argument(argc, argv, "-si-i", this->nr_iterations);
		return 1;
	};
	
	void 
	print() const
	{
		print_info("\n");
		print_highlight("Sample consensus inital alignement settings:\n");
		print_info("  Minimum sample distance:            %.2f\n", min_sample_distance);
		print_info("  Maximum correspondence distance:    %6.3f (not squared)\n", max_correspondence_distance);
		print_info("  Number of iterations:               %i\n", nr_iterations);
	};
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct voxelgrid_settings
{
	double voxsize;
	voxelgrid_settings() : voxsize(0.05) {}
	
	int
	parse(int argc, char * argv[])
	{
		parse_argument(argc, argv, "-v-vs", this->voxsize);
		return 1;
	};
	
	void
	print() const
	{
		print_info("\n");
		print_highlight("Voxelgrid parameters:\n");
		print_info("  Voxel size:                         %.2f\n", voxsize);
	};
	
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct all_settings 
{
	
	sift_settings sift;
	harris_settings harris;
	normals_settings normals;
	FPFHE_settings FPFHE;
	sac_ia_settings sac_ia;
	voxelgrid_settings voxelgrid;
	
	string target_filename;
	string source_filename;
	
	string downsampling_method;
	
	void
	print_header() const
	{
		print_info("\n");
		print_highlight("Global settings:\n");
		print_info("  Target Cloud:          %s\n", target_filename.c_str());
		print_info("  Source Cloud:          %s\n", source_filename.c_str());
		print_info("  Downsampling method:   %s\n", downsampling_method.c_str());
		print_warn("     You can change downsampling method using --sift, --harris or --voxelgrid\n");
		print_warn("     Note: none means that all points of the clouds will be used.\n");
	};
	
	void
	print() const 
	{
		print_header();
		if (downsampling_method == "sift")
		{
			sift.print();
		}
		else if (downsampling_method == "harris")
		{
			harris.print();
		}
		else if (downsampling_method == "voxelgrid")
		{
			voxelgrid.print();
		}
		
		normals.print();
		FPFHE.print();
		sac_ia.print();
		
		
	};
	
	
	void
	print_help(int argc,char* argv[]) const 
	{
		print_highlight("Syntax: %s target.pcd source.pcd [opts...]\n\n", argv[0]);
		
		print_info("Compute a first alignement between two clouds of the same scene.\n");
		print_info("If you need this help while completing your command line args just add -h and this will be printed.\n");
		print_info("Also default args and types are here exposed.\n");
		
		print_info("Alignement is achieved in several steps:\n");
		
		print_highlight(" 1. Keypoint extraction or dataset reduction. Several methods are provided:\n");
		print_info("      Just one of the following options can be chosed: --sift, --harris, --voxelgrid.\n");
		print_info("      If no options are given all the dataset will be used and a warn will be printed.\n");
		print_info("\n");
		print_info("      --sift          SIFT keypoints (use intensities), options:\n");
		print_info("        -s-mc   double    %.2f   \t Minimum constrast of features to be considered as keypoints\n", sift.min_contrast);
		print_info("        -s-ms   double    %.2f   \t Minimum scale\n", sift.min_scale);
		print_info("        -s-no   int       %i     \t Number of octaves\n", sift.nr_octaves);
		print_info("        -s-nspo int       %i     \t Number of scales per octave\n", sift.nr_scales_per_octave);
		print_info("\n");
		print_info("      --harris        Harris keypoints, options:\n");
		print_info("        -h-rs   double    %.2f   \t Search radius\n", harris.radius_search);
		print_info("        -h-r    bool      %i     \t Do refinement?\n", harris.refine);
		print_info("        -h-t    double    %.2f   \t Threshold\n", harris.threshold);
		print_info("\n");
		print_info("      --voxelgrid     Voxelgrid subsampling, options:\n");
		print_info("        -v-vs   double    %.2f   \t Size of the voxel\n", voxelgrid.voxsize);
		print_info("\n");
		print_highlight(" 2. Normal estimation, following options can be used:\n");
		print_info("        -n-ks   int       %i     \t knn  \n", normals.k_search);
		print_info("\n");
		print_highlight(" 3. FPFHE features descriptors computation, options:\n");
		print_info("      Following options are mutually exclusive. If both are given a default value will be used and a warn printed.\n");
		print_info("        -f-rs   double    %.2f   \t Search radius\n", FPFHE.radius_search);
		print_info("        -f-ks   int       %i     \t Number of neighbors for knn\n", FPFHE.k_search);
		print_info("\n");
		print_highlight(" 4. Alignement computing using sample consensus method, options:\n");
		print_info("        -si-msd double    %.2f   \t Minimum sample distance\n", sac_ia.min_sample_distance);
		print_info("        -si-mcd double    %.2f   \t Maximum correspondence distance (algorithm uses it squared. Supply it not squared here)\n", sac_ia.max_correspondence_distance);
		print_info("        -si-i   int       %i    \t Number of iterations\n", sac_ia.nr_iterations);
		
	};
	
	/**
	 * take arguments, parse them and store results in the settings structures
	 * also report errors in command line args
	 */
	int
	parse( int argc, char* argv[] )
	{
		//parse the clouds filenames
		std::vector<int> 
		file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
		
		//check arguments or print help
		if (file_indices.size() != 2)
		{
			
			print_error("You need to give 2 .pcd files as input. First is target, second source.\n");
			print_help(argc, argv);
			return -1;
		}
		
		//store the 2 paths
		target_filename = argv[file_indices[0]];
		source_filename = argv[file_indices[1]];
		
		//parse flags for subsampling/keypoint extraction
		bool is_sift = find_switch(argc, argv, "--sift");
		bool is_harris = find_switch(argc, argv, "--harris");
		bool is_voxelgrid = find_switch(argc, argv, "--voxelgrid");
		
		int sum = is_sift + is_harris + is_voxelgrid;
		if (sum > 1)
		{
			print_error("Just one method can be used for reducing the initial dataset.\n");
			print_help(argc, argv);
			return -1;
		}
		else if (sum == 0)
		{
			print_warn("No downsampling method selected. All initial dataset will be used.\n");
			downsampling_method = "none";
		}
		else //get the method name and method-specific args
	{
		if (is_sift)
		{
			downsampling_method = "sift";
			print_info("SIFT method will be used for keypoints extraction\n");
			if (sift.parse(argc, argv) == -1)
			{
				return -1;
			};
			
		}
		else if (is_harris)
		{
			downsampling_method = "harris";
			if (harris.parse(argc, argv) == -1)
			{
				return -1;
			};
		}
		else if (is_voxelgrid)
		{
			downsampling_method = "voxelgrid";
			if (voxelgrid.parse(argc, argv) == -1)
			{
				return -1;
			};
		}
	}
	
	//parse other parameters
	if (normals.parse(argc, argv) == -1)
	{
		return -1;
	}
	if (sac_ia.parse(argc, argv) == -1)
	{
		return -1;
	}
	if (FPFHE.parse(argc, argv) == -1)
	{
		return -1;
	}
	//Now we have all parameters into the settings structure!
	
	//check alse if -h is invoked and print help
	if (find_switch(argc, argv, "-h"))
	{
		print_help(argc, argv);
		return -1;
	}
	
	return 1;
	};
	
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointInT, typename PointOutT> void
compute_sift(const typename pcl::PointCloud<PointInT>::Ptr incloud_ptr, 
						 typename pcl::PointCloud<PointOutT>::Ptr out_ptr,
						 sift_settings &s
)
{
	pcl::console::print_info("Computing SIFT for cloud with %i points\n", incloud_ptr->size());
	pcl::SIFTKeypoint< PointInT, PointOutT > keypoint_detector ;
	keypoint_detector.setInputCloud(incloud_ptr);
	keypoint_detector.setScales (s.min_scale, s.nr_octaves, s.nr_scales_per_octave);
	keypoint_detector.setMinimumContrast(s.min_contrast);
	keypoint_detector.compute(*out_ptr);
	pcl::console::print_info("SIFT computed. We got %i keypoints\n", out_ptr->points.size());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
compute_harris(const typename pcl::PointCloud<PointInT>::Ptr incloud_ptr, 
							 typename pcl::PointCloud<PointOutT>::Ptr out_ptr,
							 harris_settings &s
)
{
	typename pcl::PointCloud<PointInT>::Ptr tmp (new pcl::PointCloud<PointInT>);
	pcl::console::print_info("Computing Harris for cloud with %i points\n", incloud_ptr->size());
	pcl::HarrisKeypoint3D< PointInT, PointInT > keypoint_detector ;
	keypoint_detector.setMethod(pcl::HarrisKeypoint3D< PointInT, PointInT>::NOBLE);
	
	keypoint_detector.setRadiusSearch(s.radius_search);
	keypoint_detector.setRefine(s.refine);
	keypoint_detector.setThreshold(s.threshold);
	
	keypoint_detector.setInputCloud(incloud_ptr);
	
 	keypoint_detector.compute(*tmp);
	//copy to out
	pcl::copyPointCloud<PointInT, PointOutT>(*tmp, *out_ptr);
	pcl::console::print_info("Harris computed. We got %i keypoints\n", out_ptr->points.size());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointST> void
compute_normals(const typename pcl::PointCloud<PointInT>::Ptr incloud_ptr, 
								const typename pcl::PointCloud<PointST>::Ptr surface_ptr,
								pcl::PointCloud<pcl::Normal>::Ptr normals,
								normals_settings &s
)
{
	
	//cast the surface to the input tipe
	typename pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr tmp_cloud (new pcl::PointCloud<PointT>);
	pcl::copyPointCloud<PointST, PointT>(*surface_ptr, *tmp);
	pcl::copyPointCloud<PointInT, PointT>(*incloud_ptr, *tmp_cloud);
	pcl::console::print_info("Computing %i normals\n", 
													 incloud_ptr->points.size());
	
	pcl::NormalEstimationOMP<PointT, pcl::Normal> norm_est;
	norm_est.setInputCloud (tmp_cloud);
	norm_est.setSearchSurface(tmp);
	norm_est.setKSearch (s.k_search);
	norm_est.compute(*normals);
	//print out some normals
	for (int i = 0; i < 25; ++i)
	{
		std::cout << normals->points[i] << std::endl;
		std::cout << tmp->points[i] << std::endl;
	}
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointST> void
compute_FPFH(const typename pcl::PointCloud<PointInT>::Ptr incloud_ptr,
							const typename pcl::PointCloud<PointST>::Ptr surface_ptr,
							const pcl::PointCloud<pcl::Normal>::Ptr normal_ptr,
							pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_ptr,
							FPFHE_settings &s
)
{
	
	
	
	//cast the search surface to the PointInT type
	typename pcl::PointCloud<PointInT>::Ptr tmp (new pcl::PointCloud<PointInT>);
	pcl::copyPointCloud(*surface_ptr, *tmp);
	
	
	pcl::console::print_info("Computing features descriptor for a cloud with %i points\n -> using as surface %i points\n", 
													 incloud_ptr->points.size(), 
													 surface_ptr->points.size());
	
	pcl::FPFHEstimationOMP<PointInT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud (incloud_ptr);
	fpfh_est.setSearchSurface(tmp);
	fpfh_est.setInputNormals (normal_ptr);
	if (s.search_method == "rs")
	{
		fpfh_est.setRadiusSearch (s.radius_search);
	}
	else
	{
		fpfh_est.setKSearch (s.k_search);
	}
	fpfh_est.compute(*features_ptr);
	
	//print out some histograms
	for (int i = 0; i < 25; ++i)
	{
		std::cout << features_ptr->points[i] << std::endl;
	}
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template <typename PointInT, typename PointOutT> void
compute_voxelgrid(const typename pcl::PointCloud<PointInT>::Ptr source,
									typename pcl::PointCloud<PointOutT>::Ptr out,
									voxelgrid_settings &s
)
{
	//copy the cloud in the output type
	typename pcl::PointCloud<PointOutT>::Ptr tmp (new pcl::PointCloud<PointOutT>);
	pcl::copyPointCloud<PointInT, PointOutT>(*source, *tmp);
	pcl::console::print_info("Downsampling a cloud\n");
	pcl::VoxelGrid<PointOutT> vox_grid;
	vox_grid.setInputCloud (tmp);
	vox_grid.setLeafSize (s.voxsize, s.voxsize, s.voxsize);
  vox_grid.setDownsampleAllData(1);
	vox_grid.filter(*out);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename FeatT> void
compute_sac_ia(const typename pcl::PointCloud<PointInT>::Ptr target,
							 const typename pcl::PointCloud<PointInT>::Ptr source,
							 const typename pcl::PointCloud<FeatT>::Ptr t_features,
							 const typename pcl::PointCloud<FeatT>::Ptr s_features,
							 sac_ia_settings &s,	
							 sac_ia_results &r
							)
{
	
	pcl::PointCloud<PointInT> registration_output;
	
	
	pcl::SampleConsensusInitialAlignment<PointInT, PointInT, FeatT> sac_ia;
	sac_ia.setMinSampleDistance (s.min_sample_distance);
	sac_ia.setMaxCorrespondenceDistance (s.max_correspondence_distance * s.max_correspondence_distance);
    sac_ia.setRANSACOutlierRejectionThreshold(0.6);
	sac_ia.setMaximumIterations (s.nr_iterations);
	
	sac_ia.setInputTarget (target);
	sac_ia.setTargetFeatures (t_features);
	
	sac_ia.setInputCloud (source);
	sac_ia.setSourceFeatures (s_features);
	
	sac_ia.align (registration_output);
	r.final_transformation = sac_ia.getFinalTransformation();
	r.has_converged = sac_ia.hasConverged();
	r.fitness_score = sac_ia.getFitnessScore(s.max_correspondence_distance);
	
	
}

	


	


