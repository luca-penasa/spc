#include <spc/core/strings.h>

#include <spc/methods/RBFModelEstimator.hpp>

#include <spc/io/element_io.h>
#include <spc/elements/EigenTable.h>

//#include <gflags/gflags.h>
#include <spc/core/logging.h>
#include <spc/core/flagging.h>

using namespace spc;
using Eigen::Matrix;
//namespace gflags = google;


//#ifdef GFLAGS_IN_NS_GOOGLE
//namespace gflags = google;
//#endif

DEFINE_string(out, "rbf_model", "Out Filename.");


DEFINE_string(predictor_fields, "distance,angle", "The field name of the distance field");
DEFINE_string(observation_field, "intensity", "The field name of the observation to model");

//DEFINE_string(weights_field, "intensity_std", "Used weights for solving the problem");

DEFINE_double(sigma, 0, "Sigma value for the kernel of RBF. If 0 it will be automatically chosen");
DEFINE_double(lambda, 0.01, "Lambda value for regularization (smoothness) of RBF");


DEFINE_uint64(poly_order, 1 , "polynomial order for the polynomial part of RBF");
DEFINE_string(n_nodes, "6,6", "Number of splits on each dimension (one for each predictor)"
              "The total numbe of nodes will be the product of all these splits");

DEFINE_string(input_database, "", "an input file containing the variables to be used for calibrating the model");

int main (int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

	gflags::SetUsageMessage("Compute a rbf model for predicting a given scalar field (e.g. intensity) as function of any other scalar fields."
                            "call as: " + std::string(argv[0]) + " database.spc [or xml/json] [...]");

//    FLAGS_logtostderr = 1;
    gflags::ParseCommandLineFlags(&argc, &argv, true);


    std::string datadb = FLAGS_input_database;

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

    if (FLAGS_sigma == 0 )
        estimator.autosetSigma();
    else
        estimator.getModel()->setSigma(FLAGS_sigma);

    estimator.setLambda(FLAGS_lambda);

    estimator.setInputValues(observation);


    estimator.getModel()->setPolyOrder(FLAGS_poly_order);

    CHECK(estimator.solveProblem()!= -1) << "cannot solve -- see log info please";




    LOG(INFO) << "Saving the model to " << FLAGS_out;
    spc::io::serializeToFile(estimator.getModel(), FLAGS_out, spc::io::XML);


    return 1;
}
