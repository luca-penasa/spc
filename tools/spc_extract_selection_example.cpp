#include <spc/elements/SelectionBase.h>

#include <spc/elements/PointCloudBase.h>

#include <spc/io/element_io.h>

#include <spc/methods/SelectionExtractor.h>
#include <spc/elements/SelectionBase.h>

#include <spc/io/io_helper.h>


#include <spc/elements/PointCloudPcl.h>
DEFINE_string(selection, "", "the spc file containing a selection");
DEFINE_string(cloud, "", "the point cloud file to select from");


int main(int argc, char ** argv)
{

    google::InitGoogleLogging(argv[0]);
//    std::cout << "INIZIO" << std::endl;

    FLAGS_colorlogtostderr=1;
    FLAGS_logtostderr=1;

    google::ParseCommandLineFlags(&argc, &argv, true);

    spc::ISerializable::Ptr ser =  spc::io::deserializeFromFile(FLAGS_selection);

    spc::SelectionOfPointsBase::Ptr sel = spcDynamicPointerCast<spc::SelectionOfPointsBase>(ser);


    LOG(INFO) << "selection loaded";

    spc::PointCloudBase::Ptr cloud = spc::io::loadPointCloud(FLAGS_cloud);

    LOG(INFO) << "cloud loaded";


    spc::SelectionExtractor<Eigen::Vector3f, size_t> extractor;

    extractor.setSelection(sel);
    extractor.setInputSet(cloud);
    extractor.compute();


//    LOG(INFO) << "create cloud";
//    spc::PointCloudPCL cloud;
//    LOG(INFO) << "done";


//    cloud.resize(10);
//    LOG(INFO) << cloud.asEigenMatrix();

    spc::PointCloudBase::Ptr out = cloud->extractIDS<spc::PointCloudPCL>(extractor.getInsideIds());

    pcl::io::savePCDFile("out.pcd", *out->getAsPclXyz());



//    DLOG(INFO) << "percentage " << extractor.getPercentageInside();

    return 0;
}

