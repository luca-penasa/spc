#include <spc/methods/TransferFieldNN.h>
#include <spc/core/strings.h>

#include <pcl/io/file_io.h>
#include <spc/core/flagging.h>

#include <spc/elements/PointCloudPcl.h>

#include <spc/io/io_helper.h>
#include <spc/core/filesystem.h>


DEFINE_string(from, "", "cloud from which to tranfer th fields");
DEFINE_string(to, "", "a space separated list of clouds to which transfer the fields");
DEFINE_string(fields, "", "a space separated list of fields to transfer");
DEFINE_string(postfix, "with_transferred_fields", "a postfix string to add to the output filename");

DEFINE_double(max_distance, 0.1, "nearest neighbor maximum distance considered as acceptable");

int main(int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr = 1;

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    CHECK_GT(FLAGS_max_distance, 0.0);

    DLOG(INFO) << "input string for to: \n" << FLAGS_to;


    if(FLAGS_from =="")
    {
        LOG(ERROR) << "Input cloud must be specified";
    }

    std::vector<std::string> to_cloudnames  = spc::splitStringAtSeparator(FLAGS_to, " ");
    std::vector<std::string> fields  = spc::splitStringAtSeparator(FLAGS_fields, " ");


    spc::PointCloudBase::Ptr from_cloud = spc::io::loadPointCloud(FLAGS_from);

    LOG(INFO) << "cloud loaded!";


    if (from_cloud == NULL)
    {
        LOG(FATAL) << "cannot load cloud " << FLAGS_from;
    }


    if (!from_cloud->hasFields(fields))
    {
        LOG(FATAL) << "looks like your cloud does not have the required input fields";
    }


    for (int i = 0 ; i < to_cloudnames.size(); ++i)
    {
        std::string name= to_cloudnames.at(i);
        LOG(INFO) << "working on cloud " << name;

        spc::PointCloudBase::Ptr cloud = spc::io::loadPointCloud(name);

        if(cloud == NULL)
        {
            LOG(FATAL) << "cannot load cloud " << name;
        }

        spc::PointCloudHelpers::transferFieldsNN(from_cloud, cloud, FLAGS_max_distance, fields);


        std::string newname  = spc::fs::appendPostfix(name, FLAGS_postfix, "_").string();

        LOG(INFO) << "saving as "<< newname;

        spc::io::savePointCloudAsPCDBinaryCompressed(*cloud, newname);
    }

    return 1;
}
