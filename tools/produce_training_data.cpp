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

using namespace pcl;
using namespace pcl::console;
using namespace std;

void getNearestNeighbor(const PointXYZ &point, const KdTreeFLANN<PointXYZ> &flann, float & distance, int & index)
{
    vector<int> indices;
    vector<float> sq_distances;
    flann.nearestKSearch(point, 1, indices, sq_distances);
    index = indices[0]; //first index is the neares
    distance = sqrt(sq_distances[0]);
}

void
getCorrespondingIds(const PointCloud<PointXYZ>::Ptr  reference_cloud,
                    const PointCloud<PointXYZ> & queries_cloud,
                    vector<int> &indices,
                    vector<float> &distances )
{
    //set up search structure
    pcl::KdTreeFLANN<pcl::PointXYZ> flann;
    flann.setInputCloud(reference_cloud);

    for (auto point: queries_cloud)
    {
        float distance;
        int index;

        getNearestNeighbor(point, flann, distance, index);
        indices.push_back(index);
        distances.push_back(distance);
    }
}

float
getValue(const uint point_id, const string field_name, const pcl::PCLPointCloud2 & cloud, float &value, const uint count_id=0)
{
    uint point_step = cloud.point_step;

    int field_id = getFieldIndex(cloud, field_name);
    pcl::PCLPointField field = cloud.fields.at(field_id);

//    uint count = field.count;
    uint offset = field.offset;


    memcpy(&value, cloud.data.data() + point_id * point_step + offset + count_id * sizeof(float), sizeof(float));
}

int main(int argc, char** argv)
{
    vector<int> pcd_indices = parse_file_extension_argument(argc, argv, ".pcd");
    vector<int> txt_indices = parse_file_extension_argument(argc, argv, ".svd"); //"svmlib file format"

    string features_cloud_fn = argv[pcd_indices[0]]; //first pcd are the features in a single cloud
    string geometry_cloud_fn = argv[pcd_indices[1]]; //this is the geometry cloud, used for linking xyz coord to the feature id
    string train_csv_fn = argv[txt_indices[0]]; //svn file is where to put train data

    vector<int> train_pcd_indices (pcd_indices.begin() + 2, pcd_indices.end());

    //names of files to be used for training
    vector<string> train_pcd_fns; //each cloud will be a different class for training
    for (auto id : train_pcd_indices)
        train_pcd_fns.push_back(argv[id]);

    //load geometry cloud
   pcl::PCLPointCloud2::Ptr tmp_sns (new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile(geometry_cloud_fn, *tmp_sns);

    PointCloud<PointXYZ>::Ptr geometry_cloud (new PointCloud<PointXYZ>);
    fromPCLPointCloud2(*tmp_sns, *geometry_cloud);


    std::ofstream file;
    file.open (train_csv_fn.c_str());

    std::ostringstream stream;
//    stream.precision (precision);
    stream.imbue (std::locale::classic ());

   pcl::PCLPointCloud2::Ptr features_cloud (new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile(features_cloud_fn, *features_cloud);

    int n_fields = features_cloud->fields.size(); //we consider we do not have strange paddings in fields

    unsigned int class_id = 0;
    for (auto name : train_pcd_fns) //for each class
    {


        //load the train cloud
        pcl::io::loadPCDFile(name, *tmp_sns); //load here
        PointCloud<PointXYZ> this_cloud;
        fromPCLPointCloud2(*tmp_sns, this_cloud);

        std::vector<int> indices;
        std::vector<float> distances;
        getCorrespondingIds(geometry_cloud, this_cloud, indices, distances);
        //TODO add a test on distances to check that the correspondence is ok!

        float value;
        for (auto id: indices) //for each index
        {
            int complete_field_id = 0;
            stream << class_id << " ";
            for (int field_id = 0; field_id < n_fields; ++field_id)
            {
                for (int n_mult = 0 ; n_mult < features_cloud->fields.at(field_id).count ; ++n_mult)
                {
                    getValue(id, features_cloud->fields.at(field_id).name, *features_cloud, value, n_mult);
                    if (isnan(value))
                    {
                            complete_field_id++;
                            continue;
                    }

                    stream << complete_field_id + 1  << ":" << value << " ";
                    complete_field_id++;

                }
            }

            string result = stream.str ();
            boost::trim (result);
            stream.str ("");
            file << result << "\n";
        }






        class_id+=1; //add for next class
    }

    file.close();







    return 1;
}
