
#include <gflags/gflags.h>
#include <spc/core/logging.h>
#include <spc/core/strings.h>
#include <spc/io/element_io.h>
#include <spc/io/io_helper.h>

#include <spc/elements/RBFModel.h>
#include <spc/core/filesystem.h>

using namespace spc;
DEFINE_string(predictors, "distance,angle", "The field name of the distance field");
DEFINE_string(observation, "intensity", "The field name of the observation to which apply correction");
DEFINE_string(model, "model.xml", "The RBF model for prediction");
DEFINE_string(clouds, "","the clouds to which apply correction, separated by space"  );

DEFINE_bool(split_rbf, true, "if true it will output the rbf part splitted from the polynomial part of the model");
DEFINE_bool(do_correction, true, "save also the corrected observation (the corrected intensity)");


DEFINE_bool(ascii, false, "save output as ascii file");

//! this may eb moved to PointCloudBase class itself
int getFields(const std::vector<std::string> names,
               const PointCloudBase::Ptr cloud,
               Eigen::MatrixXf & out)
{
    out.resize(cloud->getNumberOfPoints(), names.size());
    for (int i = 0; i < names.size(); ++i)
    {
        std::string fname = names.at(i);

        // check if the cloud has the field

        if (!cloud->hasField(fname))
        {
            LOG(WARNING) <<"the cloud does not contain the field " << fname;
            return -1;
        }

        Eigen::VectorXf field;
        cloud->getField(fname, field);
        out.col(i) = field;


        LOG(INFO)<< "sucessfully loaded field " << fname;
    }

    return 1; // successfull

}


int main(int argc, char ** argv)
{

    google::InitGoogleLogging(argv[0]);
    google::SetUsageMessage("Calibrate the intensity field of input cloud using the RBF model provided");

    LOG(INFO) << spc::fs::Path(argv[0]).stem();



    FLAGS_logtostderr = 1;

//    FLAGS_helpmatch = argv[0];

    google::ParseCommandLineFlags(&argc, &argv, true);

    // load the modes and chek if its ok
    spc::ISerializable::Ptr obj = spc::io::deserializeFromFile(FLAGS_model);
    if (!obj)
        LOG(FATAL) << "cannot deserialize the model "<< FLAGS_model << ", check the filename";

    // we assume the model is float!
    RBFModel<float>::Ptr model = spcDynamicPointerCast<RBFModel<float>> (obj);
    if (!model)
        LOG(FATAL) << "your model file can be loaded but it does not look like a RBF model. Check your file";


	size_t model_dimensions = model->getDimensionality();

    LOG(INFO) << "Model has " << model_dimensions << " dimensions";



    std::vector<std::string> predictors =  spc::splitStringAtSeparator(FLAGS_predictors, ",");

    for (auto s: predictors)
    {
        LOG(INFO) << "Predictor field: " << s;
    }

    CHECK (model_dimensions == predictors.size()) << "Model dimensions and user selected predictors fields mismatch";


    std::vector<std::string> clouds =  spc::splitStringAtSeparator(FLAGS_clouds, " ");

    if (clouds.size() == 0) // we use the arguments
    {
		for (int i = 1 ; i < argc; ++i)
        {
            clouds.push_back(argv[i]);
        }
    }



    for (auto s: clouds)
    {
        LOG(INFO) << "Cloud file: " << s;
    }

    CHECK (clouds.size() >= 1) << "please provide at least one cloud to correct";


    for (std::string cloudname : clouds)
    {
        LOG(INFO) << "loading cloud "<< cloudname;

        PointCloudBase::Ptr cloud = spc::io::loadPointCloud(cloudname);

        if (!cloud)
        {
            LOG(WARNING) << "cannot load the cloud. Format not recognize. check your log in debug mode \n"
                            "skipping to the next cloud (or ending if no more cloud are listed)";
            continue;
        }


        // get the needed fields
        Eigen::MatrixXf pred;
        int status = getFields(predictors, cloud, pred);
        if (status < 0)
            LOG(FATAL) <<  "some fields of predictors are missing in cloud " << cloudname << ". see log in debug mode";



        Eigen::MatrixXf  obs;
        status  = getFields({FLAGS_observation}, cloud, obs);

        if (status < 0)
            LOG(FATAL) <<  "The observation field is missing in the cloud " << cloudname << ". see log in debug mode";

        Eigen::Map<Eigen::VectorXf> ob = Eigen::Map<Eigen::VectorXf>(obs.data(), obs.size());



        Eigen::MatrixXf result;
        std::vector<std::string> result_names;

        if (FLAGS_split_rbf)
        {
            LOG(INFO) << "computing rbf model as splitted";
            model->splittedEvaluator(pred, result);
            result.conservativeResize(result.rows(), 3);
            result.col(2) = result.col(0) + result.col(1);

            result_names = {"poly_part", "rbf_part", "intensity_predicted"};

            LOG(INFO) << "Done";
        }
        else
        {
            LOG(INFO) << "computing only prediction";
            Eigen::VectorXf values;
            model->operator ()(pred, values);
            result.resize(values.rows(),1);
            result.col(0) = values;

            result_names = {"intensity_predicted"};
            LOG(INFO) << "Done";
        }


        if (FLAGS_do_correction)
        {
            LOG(INFO) << "Applying correction to the observed data";

            //add a column
            result.conservativeResize(result.rows(), result.cols() + 1);

            result.col(result.cols() - 1) = ob.array() / result.col(result.cols() - 2).array();
            result_names.push_back("intensity_corrected");

            LOG(INFO) << "Done";
        }

        //now we put everything in the origin cloud

        LOG(INFO) << "Adding the fields";
        cloud->addFields(result_names, result);


        LOG(INFO) << "Getting as PCD data";
        pcl::PCLPointCloud2::Ptr outcloud = cloud->asPCLData();


        typedef boost::filesystem::path path;
        path file_path = path(cloudname);

        std::string new_filename = file_path.stem().string() + "_calibrated.pcd";
        path outpath = file_path.parent_path() / new_filename;
        LOG(INFO) << "OUTFILE: " << outpath.string();

        LOG(INFO) << "saving ";
        pcl::io::savePCDFile(outpath.string(),*outcloud,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),!FLAGS_ascii);

        LOG(INFO) << "done";



    }


    return 1;
}
