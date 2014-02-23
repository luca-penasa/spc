#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <spc/common/common.h>
#include <spc/common/strings.h>
#include <spc/common/io_helper.h>

//#include <spc/methods/linear_interpolator.h>

#include <math.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <spc/common/pcl_helper.h>
#include <spc/common/svmlib_helper.h>



using namespace pcl;
using namespace pcl::console;
using namespace std;

typedef vector<vector<float> > stdfieldT;
typedef vector<stdfieldT> stdcloudT;

int main (int argc, char ** argv)
{
    vector<int> pcd_indices = parse_file_extension_argument(argc, argv, ".pcd");
    vector<int> svd_indices = parse_file_extension_argument(argc, argv, ".svd");

   pcl::PCLPointCloud2 cloud;
    pcl::io::loadPCDFile(argv[pcd_indices.at(0)], cloud);

    std::vector< pcl::PCLPointField>  fields = cloud.fields;

    stdcloudT all_fields;
    for (int i = 0; i < fields.size(); ++i)
    {
        pcl::PCLPointField field = fields.at(i);
        stdfieldT this_field = spc::readCompleteFieldToVector<float>(cloud, field.name);
        all_fields.push_back(this_field);
    }

    spc::writeToSVMlibFile(all_fields, argv[svd_indices.at(0)]);

    //now write all the field in a svmlib compatible ascii file


    return 1;
}
