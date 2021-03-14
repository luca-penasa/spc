
#include <spc/methods/TransferFieldNN.h>
#include <spc/core/spc_strings.h>

#include <pcl/io/file_io.h>
#include <spc/core/flagging.h>

#include <spc/elements/PointCloudPcl.h>

#include <spc/io/io_helper.h>
//#include <spc/core/filesystem.h>


DEFINE_string(clouds, "", "a space separated list of clouds for which to compute scattering angles and sensor distance");

DEFINE_string(postfix, "with_angles_distance", "a postfix string to add to the output filename");
INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);
//    FLAGS_logtostderr = 1;


	google::ParseCommandLineFlags(&argc, &argv, true);


    if(FLAGS_clouds =="")
    {
        LOG(ERROR) << "Input cloud must be specified";
    }

    std::vector<std::string> cloudnames  = spc::splitStringAtSeparator(FLAGS_clouds, " ");


    for (int i = 0 ; i < cloudnames.size(); ++i)
    {
        std::string name= cloudnames.at(i);
        LOG(INFO) << "working on cloud " << name;

        spc::PointCloudBase::Ptr cloud = spc::io::loadPointCloud(name);

        if(cloud == NULL)
        {
            LOG(ERROR) << "cannot load cloud " << name;
        }

        if (cloud->hasField("normal_x"))
            spc::PointCloudHelpers::computeScatteringAngle(cloud);


        spc::PointCloudHelpers::computeDistanceFromSensor(cloud);

        std::string newname  = spc::concatenateFilenameWithSeparator(name, FLAGS_postfix);

        LOG(INFO) << "saving as "<< newname;

        spc::io::savePointCloudAsPCDBinaryCompressed(*cloud, newname);
    }

    return 1;
}
