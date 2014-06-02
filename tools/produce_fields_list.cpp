#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <spc/methods/common.h>
#include <spc/methods/strings.h>
#include <spc/io/io_helper.h>

#include <boost/filesystem.hpp>

//#include <spc/methods/linear_interpolator.h>

#include <math.h>

using namespace pcl;
using namespace pcl::console;
using namespace std;

int fillHeader(const string & filename, pcl::PCLPointCloud2 &cloud)
{
    PCDReader r;
    r.readHeader(filename , cloud);
    return 1;
}

int main(int argc, char ** argv)
{
    vector<int> pcd_indices = parse_file_extension_argument(argc, argv, ".pcd");
    vector<int> txt_indices = parse_file_extension_argument(argc, argv, ".txt");

    if (pcd_indices.size() == 0)
    {
        print_error("Supply at least one *.pcd file\n");
        return -1;
    }

    if ((txt_indices.size() == 0) || txt_indices.size() > 1)
    {
        print_error("Supply one *.txt file for output\n");
        return -1;
    }

    string out_txt_filename = argv[txt_indices[0]]; //the first one is out file

    //open file
    std::ofstream file;
    file.open (out_txt_filename.c_str());

    //a stream
    std::ostringstream stream;
    stream.imbue (std::locale::classic ());

    BOOST_FOREACH (auto id, pcd_indices)
    {
        
		boost::filesystem::path full_path = boost::filesystem::canonical(argv[id]);
        
        boost::filesystem::path parent = full_path.parent_path().c_str();
        boost::filesystem::path only_name = full_path.filename().c_str();

		stream << parent << "/" << only_name.c_str() << "\n";

       pcl::PCLPointCloud2 incloud;
	   string qua = full_path.string();
	   fillHeader(qua, incloud);

        int n_fields  = incloud.fields.size();
        for (int i = 0; i < n_fields ; ++i)
        {
            if (incloud.fields[i].name == "_")
                    continue;

            string fieldname = incloud.fields[i].name;
            stream << i << " " <<  fieldname << "\n";

        }

    }

    //dump to file
    string result = stream.str();
    boost::trim(result);
    stream.str("");
    file << result ;
    file.close();



    return 1;
}
