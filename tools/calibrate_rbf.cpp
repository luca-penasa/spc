#include <spc/methods/strings.h>

#include <spc/methods/RBFModelEstimator.hpp>

#include <spc/io/element_io.h>
#include <spc/elements/EigenTable.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace spc;
using Eigen::Matrix;


DEFINE_string(out, "model.xml", "Out Filename.");


DEFINE_string(predictor_fields, "distance,intensity", "The field name of the distance field");
DEFINE_string(observation_field, "intensity", "The field name of the observation to model");

//DEFINE_string(weights_field, "intensity_std", "Used weights for solving the problem");

DEFINE_double(sigma, 0, "Sigma value for the kernel of RBF. If 0 it will be automatically chosen");
DEFINE_double(lambda, 0.01, "Lambda value for regularization (smoothness) of RBF");


DEFINE_uint64(poly_order, 1 , "polynomial order for the polynomial part of RBF");
DEFINE_string(n_nodes, "6,6", "Number of splits on each dimension (one for each predictor)"
              "The total numbe of nodes will be the product of all these splits");


int main (int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr = 1;


    google::ParseCommandLineFlags(&argc, &argv, true);



    CHECK(argc == 2) << "please provide an argument";

    std::string datadb = argv[1];

    LOG(INFO) << "working on file " << datadb;




    LOG(INFO) << "calibrating a linear model";

    ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

    CHECK(o != NULL) << "cannot deserialize the file";

    EigenTable::Ptr table_ = spcDynamicPointerCast<EigenTable> (o);

    std::vector<std::string> fields = splitStringAtSeparator(FLAGS_predictor_fields, ",");

    table_ = table_->getWithStrippedNANs(fields);


    std::vector<Eigen::VectorXf> predictors;
    for (auto f: fields)
    {
        LOG(INFO) << "Predictor Field: " << f;
        predictors.push_back(table_->column(f));
    }

    CHECK(predictors.size() == fields.size()) << "cannot find all the fields we need. Check your flags";




    Eigen::MatrixXf points(table_->getNumberOfRows(), fields.size());
    for (int i = 0; i < predictors.size(); ++i)
    {
        points.col(i) = predictors.at(i);
    }

    predictors.clear();


    Eigen::VectorXf observation = table_->column(FLAGS_observation_field);

    CHECK(observation.size() != 0) << "some problem locating the observation fields in the dataset";



    std::vector<std::string> nod = splitStringAtSeparator(FLAGS_n_nodes, ",");
    CHECK(nod.size() == points.cols()) << "You need to specify a number of splits for EACH predictor variable";

    Eigen::VectorXi n_splits(nod.size());

    size_t i = 0;
    for (auto s: nod)
    {
        n_splits(i++) = atoi(s.c_str());
    }

    spc::RBFModelEstimator<float> estimator;
    estimator.setPoints(points);
    estimator.autosetScales(0);
    estimator.autosetNodes(n_splits);

    estimator.setLambda(FLAGS_lambda);

    estimator.setInputValues(observation);


    estimator.getModel()->setPolyOrder(FLAGS_poly_order);
    estimator.getModel()->setSigma(FLAGS_sigma);

    CHECK(estimator.solveProblem()!= -1) << "cannot solve -- see log info please";

    //        LinearCalibrator calibrator(datadb);


    //        std::string outfname = "/home/luca/test_linear.txt";

    //        SampledData * data_ptr = &calibrator.getData();

    //        Eigen::VectorXf pred = calibrator.getPredictionForInput();

    //        EigenTable::Ptr out (new EigenTable);
    //        out->addNewComponent("distance", 1);
    //        out->addNewComponent("angle", 1);
    //        out->addNewComponent("intensity", 1);
    //        out->addNewComponent("pred_intensity", 1);




    //        out->resize(data_ptr->i_.size());

    //        out->column("distance") = data_ptr->d_;
    //        out->column("angle") = data_ptr->a_;
    //        out->column("intensity") = data_ptr->i_.cast<float>();
    //        out->column("pred_intensity") = pred.cast<float>();


    //        spc::io::AsciiEigenTableWriter w;
    //        w.setInput(out);
    //        w.setOutputFilename(outfname);
    //        w.setWriteHeaders(true);
    //        w.write();

    return 1;
}
