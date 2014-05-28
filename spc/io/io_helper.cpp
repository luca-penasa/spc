#include "io_helper.h"
#include <pcl/io/pcd_io.h>

#include <spc/methods/common.h>


using namespace pcl::console;
using namespace pcl;


namespace spc
{
int 
loadCSVFile(const std::string &in_filename, pcl::PCLPointCloud2 &output, const int x_id, const int y_id, const int z_id, const int i_id, const int k, const std::string s)
{
	//open file
    std::ifstream in(in_filename.c_str());
	if (!in.is_open()) 
	{
        print_error("Error loading file\n");
		return -1;
	}
	
	
	
	PointCloud<PointXYZI>::Ptr cloud (new PointCloud<PointXYZI>);
	
	int n_lines = 0;
	
    std::string line;
    std::vector< std::string > vec;
	while ( in.eof() != 1 )
	{
		//Always get the line -> so while advances
        std::getline(in, line);
		// 		cout << line << endl;
		
		//skip a number k of rows
		n_lines++;
		if (n_lines <=  k)
		{
			continue;
		}
		
		//void lines could be accientaly present, in case continue
		if (line.size() == 0) 
		{
			continue;
		}
		
		
		//Tokenize the line
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		boost::char_separator<char> sep(s.c_str());
		tokenizer tokens(line, sep);
		
		//Assign tokens to a string vector
		vec.clear();
		vec.assign(tokens.begin(), tokens.end());
		
		
		//now check if we have a column for each requested field
		if (vec.size() < x_id)
		{
			PCL_ERROR("You requested a x field that does not exist in your file! Check separtor too.\n");
			PCL_ERROR("Error at line %i.\n", n_lines);
			return(-1);
		}
		
		if (vec.size() < y_id)
		{
			PCL_ERROR("You requested a y field that does not exist in your file! Check separtor too.\n");
			PCL_ERROR("Error at line %i.\n", n_lines);
			return(-1);
		}
		if (vec.size() < z_id)
		{
			PCL_ERROR("You requested a z field that does not exist in your file! Check separtor too.\n");
			PCL_ERROR("Error at line %i.\n", n_lines);
			return(-1);
		}
		if (vec.size() < i_id)
		{
			PCL_ERROR("You requested an intensity field that does not exist in your file! Check separtor too.\n");
			PCL_ERROR("Error at line %i.\n", n_lines);
			return(-1);
		}
		
		//just some verbosity
        if (n_lines % 100000 == 0)
		{
			print_info("line %i\n", n_lines);
		}
		
		//Create the point
		PointXYZI point;
		point.x = atof(vec[x_id-1].c_str());
		point.y = atof(vec[y_id-1].c_str());
		point.z = atof(vec[z_id-1].c_str());
		point.intensity = atof(vec[i_id-1].c_str());
		
		//Add the point to the cloud
		cloud->push_back (point); 	
	}
	
	//Now resize the cloud
	int n_points = n_lines - k -1;
	
	cloud->width = n_points;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	
    pcl::toPCLPointCloud2(*cloud, output);

    return 1;
	
}


int
loadCSVTimeSeries(const std::string in_filename, const std::string separator, std::vector< std::vector<float> > &tseries)
{
    tseries.resize(2); //this is made only for 2d time-series
    //open file
    std::ifstream in(in_filename.c_str());
    if (!in.is_open())
    {
        print_error("Error loading file\n");
        return -1;
    }


    std::string line;
    while ( in.eof() != 1 )
    {
        std::getline(in, line);

        //Tokenize the line
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep(separator.c_str());
        tokenizer tokens(line, sep);

        //Assign tokens to a string vector
        std::vector<std::string> vec;
        vec.clear();
        vec.assign(tokens.begin(), tokens.end());

        if (vec.size() == 0) //void lines?
            continue;

        //put the first two elements in output
        tseries[0].push_back(atof(vec[0].c_str()));
        tseries[1].push_back(atof(vec[1].c_str()));
    }


    return 1;
}

template<typename T>
int
saveAsCSV(const std::string &filename, const std::string &separator, const std::vector< std::vector<T> > &columns, const int precision)
{
    int n_columns = columns.size();
    int n_rows = columns.at(0).size();

    std::ofstream file;
    file.open (filename.c_str());

//    // Mandatory lock file
//    boost::interprocess::file_lock file_lock;
//    pcl::io::setLockingPermissions (filename.c_str(), file_lock);


    std::ostringstream stream;
    stream.precision (precision);
    stream.imbue (std::locale::classic ());

    for (int i = 0; i < n_rows; ++i)
    {
        for (int c=0; c < n_columns; ++c)
        {

            T value = columns.at(c).at(i);

            if (pcl_isnan(value))
                stream << "nan";
            else
                stream << value;

            if (c != n_columns-1)
                stream << separator.c_str();

        }

        //stream now contains the line

        std::string result = stream.str ();
        boost::trim (result);
        stream.str ("");
        file << result << "\n";
    }

    file.close();
//    pcl::io::resetLockingPermissions (file_name, file_lock);

    return 1;
}


int
savePCDBinaryCompressed(const std::string &filename, const pcl::PCLPointCloud2 &cloud)
{
    pcl::PCDWriter w;
    return w.writeBinaryCompressed(filename, cloud);

}

template
int
saveAsCSV(const std::string &filename, const std::string &separator, const std::vector<std::vector<float> > &columns, const int precision);

template
int
saveAsCSV(const std::string &filename, const std::string &separator, const std::vector<std::vector<double> > &columns, const int precision);


} //end nspace
