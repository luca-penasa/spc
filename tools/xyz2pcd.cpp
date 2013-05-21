
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <spc/common/strings.h>
#include <pcl/console/parse.h>


#include <algorithm>    // copy
#include <iterator>     // ostream_operator
#include <iostream>     // cout, endl
#include <fstream>      // fstream
#include <vector>


#include <spc/common/std_helpers.hpp>

#include <spc/common/io_helper.h>


using namespace pcl::io;
using namespace pcl::console;
using namespace pcl;
using namespace boost;
using namespace std;


string separator_default = ";";
int int_column_default = 4;
int x_column_default = 1;
int y_column_default = 2;
int z_column_default = 3;
int skip_n_rows_default = 0;
int binary_default = 1;


void 
printHelp(int argc, char *argv[])
{
	print_error("Syntax is ...\n");
	print_info("Columns are numbered from 1 to N (first column is 1)\n");
	print_info("Filename is automatically renamed from input.csv [or txt, ...] to input.pcd\n\n");
	
	print_info("Options are:\n");
	print_info("            -k number of rows to skip reading the file.     Default %i\n", skip_n_rows_default);
	print_info("            -s \"char\" separator, between quotes.            Default \"%s\" \n", separator_default.c_str());
	print_info("            -x N, column to be used for the x coordinate.   Default %i \n", x_column_default);
	print_info("            -y N, column to be used for the y coordinate.   Default %i \n", y_column_default);
	print_info("            -z N, column to be used for the z coordinate.   Default %i \n", z_column_default);	
	print_info("            -i N, column to be used as intensity.           Default %i \n", int_column_default);
	print_info("            -b 0/1, save as binary pcd or not.              Default %i \n", binary_default);

}

void
saveCloud (const std::string &filename, const sensor_msgs::PointCloud2 &output, bool is_binary)
{
//   TicToc tt;
//   tt.tic ();

  print_highlight ("Saving "); print_value ("%s \n", filename.c_str ());
	pcl::io::savePCDFile (filename, output, Eigen::Vector4f::Zero (),Eigen::Quaternionf::Identity (), is_binary);	
	



//   print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms: "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}


// std::string stripExtension(const std::string& fileName)
// {
//   
//   for (int i=fileName.length(); i > 0; --i)
//   {
//     if (fileName[i]=='.') 
//     {
//       return fileName.substr(0, i);
//     }
//   }
//   return fileName;
// }




int main(int argc, char *argv[] )
{
	if (argc < 2) // arguments need at least to be 1! else help
	{
		printHelp (argc, argv);
		return (-1);
	}
	
	//Load default settings
	int x_column = x_column_default;
	int y_column = y_column_default;
	int z_column = z_column_default;
	int int_column = int_column_default;
	std::string separator = separator_default;
	int skip_n_rows = skip_n_rows_default;
	int binary = binary_default;
	
	//And parse
	parse_argument (argc, argv, "-x", x_column);
	parse_argument (argc, argv, "-y", y_column);
	parse_argument (argc, argv, "-z", z_column);
	parse_argument (argc, argv, "-i", int_column);
	parse_argument (argc, argv, "-k", skip_n_rows);
	parse_argument (argc, argv, "-s", separator);
	parse_argument (argc, argv, "-b", binary);
	
	//Also get input files
	vector<int> csv_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".csv");
	vector<int> txt_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".txt");
	vector<int> xyz_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".xyz");
	
	//put filenames on just one vector
	vector <string> file_names;
	for (int i = 0; i < csv_file_indices.size(); i++)
	{
		file_names.push_back(argv[csv_file_indices.at(i)]);
	}
	
	for (int i = 0; i < txt_file_indices.size(); i++)
	{
		file_names.push_back(argv[txt_file_indices.at(i)]);
	}
	
	for (int i = 0; i < xyz_file_indices.size(); i++)
	{
		file_names.push_back(argv[xyz_file_indices.at(i)]);
	}
	
	//Check if we have a good input filename 
	if (file_names.size() == 0)
	{
		print_error("ERROR: Your filename MUST have a supported extension\n");
		printHelp(argc, argv);
		return(1);
	}
	
	
	//Print out the filenames
	for (int n = 0; n < file_names.size(); n++)
	{
		print_info("Going to convert following files:\n");
		print_value("  -> %s\n", file_names[n].c_str());
	}
	
	//For each file:
	for (int n = 0; n < file_names.size(); n++)
	{
		string filename  = file_names.at(n);
		
		//Open the file 
		ifstream in(filename.c_str());
		if (!in.is_open())
		{
			print_error("Some problem occurred opening the file %s\n", filename.c_str());
			return (-1);
		}
		
		//Set up the blob 
		sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
		
		//get the cloud
        int status = spc::loadCSVFile(filename, *cloud, x_column, y_column, z_column, int_column, skip_n_rows, separator);

		if (status == -1)
		{
			print_error("Some error occured. Check above log please.\n");
			return (-1);
		}
		
		//strip extension
        string just_name = spc::stripExtension(filename);
		print_info("stripped name: %s\n", just_name.c_str());
		
		//change extension
		string outfilename = just_name + ".pcd";
		print_info("new name %s\n", outfilename.c_str());
		
		bool is_binary;
		if (binary == 0)
		{
			is_binary = false;
		}
		else if (binary == 1)
		{
			is_binary = true;
		}
		else
		{
			print_error("Accepted options for -b is 1 (binary mode) or 0 (ascii mode)");
			return (-1);
		}
		//Save cloud
		saveCloud(outfilename, *cloud, is_binary);
		
	}
	
	
    

    }
	
