#include <spc/core/strings.h>

#include <spc/methods/RBFModelEstimator.hpp>

#include <spc/io/element_io.h>
//#include <spc/elements/EigenTable.h>

#include <spc/core/flagging.h>
#include <spc/core/logging.h>

#include <spc/elements/NewSpcPointCloud.h>
#include <spc/methods/IntensityCalibratorRBF.h>
#include <spc/methods/IntensityCalibratorRBFNonLinear.h>
using namespace spc;
using Eigen::Matrix;


DEFINE_string(out, "rbf_model", "Out Filename.");


//DEFINE_string(weights_field, "intensity_std", "Used weights for solving the problem");

DEFINE_double(sigma, 0, "Sigma value for the kernel of RBF. If 0 it will be automatically chosen");
DEFINE_double(lambda, 0, "Lambda value for regularization (smoothness) of RBF"
                         "If 0 no regularization applied");

DEFINE_bool(angles, true, "If calibrate the model to account for scattering angles");

DEFINE_bool(weights, true, "Uses weighted least squres for solving the model");

DEFINE_bool(undef_mat, true, "Append additional constrains for keypoints seen by "
                             "different point of views but of unknown materials. "
                             "Not granted to provide better results. (see also -adds_mat flag)");

DEFINE_double(shift, 0, "An overall shift to be added to the measured intensities before to do anything");

DEFINE_double(undef_mat_w, 1, "Overall weight for the undefined materials. Tweak this to give more or less"
			  "importance to the observations for which a material was not specified.");

DEFINE_bool(adds_mat, true, "append additional constraints for the additional materials provided "
                            "notice that additional materials can be provided but it is likely that "
                            "the law controllig angle-effect it is not the same for the materials. "
                            "so this does not automatically provides better results. ");

DEFINE_double(adds_mat_w, 1, "Overall weight for the additional materials. Tweak this to give more or less"
			  "importance to the additional materials.");

DEFINE_uint64(n_dist, 6, "Number of nodes along the angles for constructing the rbf model");
DEFINE_uint64(n_ang, 6,  "Number of nodes along the angles for constructing the rbf model"
                        "Notice that the total number of nodes will be n_dist * n_ang. so be careful.");

DEFINE_uint64(poly_order, 1 , "polynomial order for the polynomial part of RBF");


DEFINE_uint64(init_set, 0 , "The id of the material to be used as init set for the least squares problem");


DEFINE_string(input_database, "", "an input file containing the variables to be used for calibrating the model");


DEFINE_bool(sample_model, true, "sample the calibrated model in a regular grid fashion for diaplying the surface. "
                                "a txt file with the data will be saved");

DEFINE_bool(init_data_correction, true, "sample the calibrated model at the points defined by the material 0. "
                                        "a txt file with the data will be saved");

DEFINE_bool(nonlin, false, "Do an additiona nonlinear optimization step to oeptimize the rbf model against additional constraints");

INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);
    google::SetUsageMessage("Compute a rbf model for predicting a given scalar field (e.g. intensity) as function of any other scalar fields."
                            "call as: " + std::string(argv[0]) + " database.spc [or xml/json] [...]");

//    FLAGS_logtostderr = 1;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_input_database == "")
        FLAGS_input_database = argv[1]; // get it from the first arg


    std::string datadb = FLAGS_input_database;

    LOG(INFO) << "working on file " << datadb;




    LOG(INFO) << "calibrating a linear model";

    ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

    CHECK(o != NULL) << "cannot deserialize the file";

	calibration::DataHolder::Ptr cal_data = spcDynamicPointerCast<calibration::DataHolder> (o);


    LOG(INFO) << "going do perform calibration";
    calibration::IntensityCalibratorRBF calibrator;
	calibrator.setUseWeights(FLAGS_weights);
    calibrator.setCalibrationData(cal_data);
    calibrator.setNAngleSplits(FLAGS_n_ang);
    calibrator.setNDistanceSplits(FLAGS_n_dist);
    calibrator.setAppendAdditionalMaterialsConstraints(FLAGS_adds_mat);
    calibrator.setAppendUndefMaterialsConstraints(FLAGS_undef_mat);
    calibrator.setManualSigma(FLAGS_sigma);
    calibrator.setLambda(FLAGS_lambda);
	calibrator.setAdditionalMaterialsWeight(FLAGS_adds_mat_w);
	calibrator.setUndefMaterialsWeight(FLAGS_undef_mat_w);
	calibrator.setPolyOrder(FLAGS_poly_order);
	calibrator.setInitSetMaterial(FLAGS_init_set);
	calibrator.setIntensiytShift(FLAGS_shift);

    LOG(INFO) << "computing";

    calibrator.calibrate();

    spc::RBFModel<float>::Ptr model = calibrator.getModel();

	CHECK(model != NULL) << "null ptr as model";



	spc::io::serializeToFile(calibrator.getModel(), FLAGS_out, spc::io::XML);



	if (FLAGS_nonlin)
	{

//		model->resetCoefficients(0);

		calibration::IntensityCalibratorRBFNonLinear optimizer;
		optimizer.setModel(model);
		optimizer.setCalibrationData(cal_data);
		optimizer.setFixedMaterialId(FLAGS_init_set);
//		optimizer.updateSecondary();
		optimizer.optimize();

		model = optimizer.getModel();

		std::string nonlinoptname = FLAGS_out + "_nonlin_opt";

		LOG(INFO) << "Saving the model to " << FLAGS_out;
		spc::io::serializeToFile(calibrator.getModel(), nonlinoptname, spc::io::XML);

	}








    if (FLAGS_sample_model)
    {
        LOG(INFO) << "performing sampling of the model on a regular grid";

        NewSpcPointCloud::Ptr asc = cal_data->asPointCloud();

        Eigen::Matrix<float, -1, -1> sample_pts;
        Eigen::VectorXf d = Eigen::VectorXf::LinSpaced(100,
                                                       asc->getFieldByName("distance").minCoeff(),
                                                       asc->getFieldByName("distance").maxCoeff());


        if (model->getNodes().cols() == 2) // we are using also the angles
        {
            Eigen::VectorXf a = Eigen::VectorXf::LinSpaced(100,
                                                           asc->getFieldByName("angle").minCoeff(),
                                                           asc->getFieldByName("angle").maxCoeff());

            sample_pts = meshgrid<float>({d, a});

        }
        else
        {
            sample_pts.resize(d.rows(), Eigen::NoChange);
            sample_pts.col(0) = d;
        }

        Eigen::VectorXf out;
        model->operator ()(sample_pts, out);

        LOG(INFO) << "sampling done";

        NewSpcPointCloud::Ptr outdata (new NewSpcPointCloud);
        outdata->addNewField("distance", 1);
        outdata->conservativeResize(sample_pts.rows());
        outdata->getFieldByName("distance") = sample_pts.col(0);

        if (model->getNodes().cols() == 2) // we are using also the angles
        {
            outdata->addNewField("angle", 1);
            outdata->getFieldByName("angle") = sample_pts.col(1);
        }


        outdata->addNewField("prediction", 1);
        outdata->getFieldByName("prediction") = out;



        spc::io::serializeToFile(outdata,"sampled_model", spc::io::ASCII);

    }

    if (FLAGS_init_data_correction)
    {
        LOG(INFO) << "performing correction of the input data -  only init material";

        //! only points on mat0
        NewSpcPointCloud::Ptr asc = cal_data->getKeypointsOnMaterial(0)->asPointCloud();

        Eigen::Matrix<float, -1, -1> sample_pts(asc->getNumberOfPoints(), 1);
        sample_pts.col(0) = asc->getFieldByName("distance");

        if (model->getNodes().cols() == 2) // we are using also the angles
        {
            sample_pts.conservativeResize(Eigen::NoChange, 2);
            sample_pts.col(1) =  asc->getFieldByName("angle");

        }


        Eigen::VectorXf out;
        model->operator ()(sample_pts, out);

        LOG(INFO) << "sampling done";

        NewSpcPointCloud::Ptr outdata (new NewSpcPointCloud);
        outdata->addNewField("distance", 1);
        outdata->conservativeResize(sample_pts.rows());
        outdata->getFieldByName("distance") = sample_pts.col(0);

        if (model->getNodes().cols() == 2) // we are using also the angles
        {
            outdata->addNewField("angle", 1);
            outdata->getFieldByName("angle") = sample_pts.col(1);
        }

        outdata->addNewField("intensity", 1);
        Eigen::VectorXf orig_i = asc->getFieldByName("intensity");
        outdata->getFieldByName("intensity") = orig_i;



        asc->addNewField("prediction", 1);
        asc->getFieldByName("prediction") = out;



        spc::io::serializeToFile(asc,"init_data_correction", spc::io::ASCII);

    }



    return 1;
}
