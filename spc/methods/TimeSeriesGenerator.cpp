#include <spc/methods/TimeSeriesGenerator.h>
//#include <spc/io/pointcloud2_reader.h>
//#include <spc/methods/KernelSmoothing.h>
#include <spc/elements/TimeSeriesSparse.h>
//#include <pcl/console/print.h>
#include <spc/methods/KernelSmoothing2.h>

#include <spc/methods/RBFModelEstimator.h>

namespace spc
{

int TimeSeriesGenerator::compute()
{

    if (y_field_name_.empty() && !colors_instead_of_field_) {
        LOG(ERROR) << "No scalar field name specified as values";
        return -1;
    }

    if (!in_cloud_) {
        LOG(ERROR) << "No input cloud";
        return -1;
    }

    // extract fields before to check
    if (extractFields() != 1)
    {
        LOG(INFO) << "Error in fields extraction. Maybe we did not found any point in the selection? ";
        return -1;
    }


    // now check
    if ((!model_) && (x_field_.empty() || y_field_.empty())) {
        LOG(ERROR) << "No input model or no x/y field names ";
        return -1;
    }

    if (do_autocalibration_)
    {
        DLOG(INFO) << "everything ok going to perform autocalibration";
        spc::RBFModelEstimator<float> estimator;

        int col_n  = 1;
        if (angle_field_.size() != 0)
            col_n++;

        Eigen::MatrixXf vars(distance_field_.rows(), col_n);
        vars.col(0) = distance_field_;
        if (angle_field_.size() != 0)
            vars.col(1) = angle_field_;



        estimator.setPoints(vars);
        estimator.autosetScales(0); // fixing the distance as scale


        // configuring the splits
        Eigen::VectorXi nsplits;
        nsplits.resize(1);
        nsplits(0)  = n_distance_splits_;
        if (angle_field_.size() !=0)
        {
            nsplits.conservativeResize(2);
            nsplits(1) = n_angle_splits_;
        }

        estimator.autosetNodes(nsplits);


        estimator.autosetSigma();


        estimator.setLambda(0.1);

        estimator.setInputValues(y_field_);


        estimator.getModel()->setPolyOrder(1);

        CHECK(estimator.initProblem() != -1) << "problem initializing the calibration task";

        CHECK(estimator.solveProblem()!= -1) << "cannot solve -- see log info please";

        LOG(INFO) << "autocalibration computed. Correcting intensities";

        RBFModel<float>::Ptr model = estimator.getModel();
        Eigen::VectorXf predicted;
        model->operator ()(vars, predicted);

        // apply correction
        y_field_ = y_field_.array() / predicted.array();

        LOG(INFO) << "model correctly applied";
    }
    KernelSmoothing<ScalarT> ks(x_field_, y_field_);
    ks.setKernelSigma(bandwidth_);
    ks.setMinNumberOfNeigbors(m_min_number_points);

    // and also init the output series
    if (min_x_ != max_x_)
    {
        // fixed min-max length series
        out_series_ = TimeSeriesEquallySpaced
            ::Ptr(new TimeSeriesEquallySpaced
                           (min_x_, max_x_, sampling_step_));

    } else
    {
        out_series_ = TimeSeriesEquallySpaced::Ptr(
            new TimeSeriesEquallySpaced
            (x_field_.minCoeff(), x_field_.maxCoeff(), sampling_step_));
    }

    int status = ks(out_series_->getX(), out_series_->getY());

    if (!status)
    {
        LOG(ERROR) << "Some error occured while genrating the ts";
        return -1;
    }

    return 1;
}

void TimeSeriesGenerator::fillIndicesIfNeeded()
{
    if (indices_.size() == 0)
        for (int i = 0; i < in_cloud_->getNumberOfPoints(); ++i)
            indices_.push_back(i);
}

int TimeSeriesGenerator::extractFields()
{

    if (selection_ != NULL)
    {
        LOG(INFO) << "Extracting selection";
        SelectionExtractor<Eigen::Vector3f, size_t> extractor;
        extractor.setSelection(selection_);
        extractor.setInputSet(in_cloud_);
        extractor.compute();
        indices_ = extractor.getInsideIds();
        if (indices_.size() < 10)
        {
            return -1;
        }

        LOG(INFO) << "extracted the " << extractor.getPercentageInside() << "% of points";
    }
    else
    {
        LOG(INFO) << "no selection found";
    }

    fillIndicesIfNeeded();

    if (do_autocalibration_)
    {

        LOG(INFO) << "going to compute an automatic calibration";
        // get the fields used in the autocalibration process
        if (in_cloud_->hasField(distance_field_name_))
        {
            in_cloud_->getField(distance_field_name_, indices_, distance_field_);
            LOG(INFO) << "distance field found";
        }

        if (in_cloud_->hasField(angle_field_name_))
        {
            in_cloud_->getField(angle_field_name_, indices_, angle_field_);
            LOG(INFO) << "angle field found";
        }

        LOG(INFO) << "fields for auto calibration extracted from cloud";

    }



    if (!x_field_name_.empty())
        in_cloud_->getField(x_field_name_, indices_, x_field_);

    else // we should have a stratigrahic model
    {
        x_field_ = evaluate_dynamic_scalar_field_generator<float,size_t>(in_cloud_,  model_, indices_);
    }


    // now the y field
    if (colors_instead_of_field_) // using colors
        in_cloud_->getRGBField(color_channel_,y_field_, indices_);
    else
        in_cloud_->getField(y_field_name_, indices_, y_field_);



    return 1;
}

} // end nspace
