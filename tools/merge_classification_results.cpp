#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <spc/common/common.h>
#include <spc/common/strings.h>
#include <spc/common/io_helper.h>

#include <spc/methods/linear_interpolator.h>
#include <math.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <spc/common/pcl_helper.h>
#include <boost/algorithm/string.hpp>

using namespace pcl;
using namespace pcl::console;
using namespace std;

struct PointId
    {
        unsigned int id;




        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT   (PointId,
                         (int, id, id)


    )

    struct PointProb
        {
            float p1;
            float p2;




            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EIGEN_ALIGN16;

        POINT_CLOUD_REGISTER_POINT_STRUCT   (PointProb,
                             (float, p1, p1)
                             (float, p2, p2)



        )

        void printHelp(int argc, char ** argv)
        {
            print_info("");
        }


int main(int argc, char **argv)
{
    bool probabilites = false;

    vector<int> pcd_indices = parse_file_extension_argument(argc, argv, ".pcd");
    vector<int> txt_indices = parse_file_extension_argument(argc, argv, ".txt");

    parse_argument(argc, argv, "-b", probabilites);
    if (probabilites)
        print_highlight("adding proabilities to output\n");

    //TODO add a check to verify probabilities exist in input file please

    std::ifstream file;
    file.open (argv[txt_indices[0]]);

    string line;
    PointCloud<PointId> cloud;
    PointId point;

    PointCloud<PointProb> cloud_prob;
    PointProb point_prob;


    std::vector<std::string> strs;

    while (file.eof() != 1)
    {
        std::getline(file, line);


        if (line.empty()) //this sometimes happen in the end of a file
                continue;
        //split
        boost::split(strs, line, boost::is_any_of(" "));

        if (strs.at(0) == std::string("labels") ) //when estimating prob libsvm adds an header row to output!
            continue;

        point.id = atoi(strs.at(0).c_str());
        if (probabilites)
        {
            point_prob.p1 = atof(strs.at(1).c_str());
            point_prob.p2 = atof(strs.at(2).c_str());
            cloud_prob.push_back(point_prob);
        }
        cloud.push_back(point);

    }

    sensor_msgs::PointCloud2 sns_cloud;
    toROSMsg(cloud, sns_cloud);

    sensor_msgs::PointCloud2 sns_cloud_prob;
    toROSMsg(cloud_prob, sns_cloud_prob);


    sensor_msgs::PointCloud2 full_cloud, out_cloud;
    pcl::io::loadPCDFile(argv[pcd_indices[0]], full_cloud);



    concatenateFields(full_cloud, sns_cloud, out_cloud);
    if (probabilites)
    {
        concatenateFields(sns_cloud_prob, out_cloud, full_cloud); //reusing full_cloud!
        spc::savePCDBinaryCompressed(argv[pcd_indices[1]], full_cloud);
    }
    else
    {
        spc::savePCDBinaryCompressed(argv[pcd_indices[1]], out_cloud);
    }










    return 1;
}
