#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <pcl/io/pcd_io.h>


#include <pcl/filters/extract_indices.h>

#include <spc/io/io_helper.h>

using namespace pcl;
using namespace std;
using namespace pcl::console;


struct PointId
    {

        unsigned int id;




        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT   (PointId,

                         (int, id, id)


    )


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

    BOOST_FOREACH (auto point, queries_cloud)
    {
        float distance;
        int index;

        getNearestNeighbor(point, flann, distance, index);
        indices.push_back(index);
        distances.push_back(distance);
    }
}

INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);

    vector<int> pcd_files_indices = parse_file_extension_argument(argc, argv, ".pcd"); //first is input second queries point


    string ids_file;
    parse_argument(argc, argv, "-id", ids_file);
   pcl::PCLPointCloud2 original_cloud;
    pcl::io::loadPCDFile(argv[pcd_files_indices[0]], original_cloud); //load first cloud

   pcl::PCLPointCloud2 query_cloud;
    pcl::io::loadPCDFile(argv[pcd_files_indices[1]], query_cloud); //load the second as gemetric query cloud

    //to PCL xyz
    PointCloud<PointXYZ>::Ptr original_cloud_pcl (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr query_cloud_pcl (new PointCloud<PointXYZ>);




    if (ids_file.empty())
    {
        //load only if really needed
        fromPCLPointCloud2(original_cloud, *original_cloud_pcl);
        fromPCLPointCloud2(query_cloud, *query_cloud_pcl);
    }

    vector<int> indices;
    vector<float> distances;

    PointCloud<PointId> ids_cloud;



    if (ids_file.empty())
        getCorrespondingIds(original_cloud_pcl, *query_cloud_pcl, indices, distances);
    else
    {
        print_highlight("Using %s as ids file\n", ids_file.c_str());
        pcl::io::loadPCDFile(ids_file, ids_cloud);
        //put these ids into the indices vector
        indices.resize(ids_cloud.size());
        std::transform(ids_cloud.begin(), ids_cloud.end(), indices.begin(), []( PointId &point){return point.id;});

    }

    PointIndices::Ptr indices_pcl (new PointIndices);
    indices_pcl->indices = indices;


    ExtractIndices<pcl::PCLPointCloud2> extractor;
    extractor.setIndices(indices_pcl);

    boost::shared_ptr<pcl::PCLPointCloud2> original_cloud_ptr = boost::make_shared<pcl::PCLPointCloud2>(original_cloud);
    extractor.setInputCloud(original_cloud_ptr);

   pcl::PCLPointCloud2 new_cloud;
    extractor.filter(new_cloud);

    string outfilename = argv[pcd_files_indices[2]];
    spc::io::savePCDBinaryCompressed(outfilename, new_cloud);



    return 0;
}
