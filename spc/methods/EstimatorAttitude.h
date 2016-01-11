#ifndef SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H
#define SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H

#include <spc/core/spc_eigen.h>

#include <spc/elements/StratigraphicModelSingleAttitude.h>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <spc/methods/DynamicScalarFieldEvaluator.h>
#include <spc/elements/PointCloudPcl.h>
#include <numeric>

#include <iostream>

namespace spc
{

/// a templated version of the generic functor to be used in nonlinear
/// optimizations
template <typename _Scalar, int NX = Dynamic, int NY = Dynamic> struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix
        <Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
    {
    }
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values)
    {
    }

    int inputs() const
    {
        return m_inputs;
    }
    int values() const
    {
        return m_values;
    }
};

///
/// \brief The SinglePlaneModelFromMultiCloudEstimator class
///
template <typename ScalarT>
class AttitudeEstimator
{
public:
    typedef Eigen::Matrix<ScalarT, 3, 1> PointT;
    typedef Eigen::Matrix<ScalarT, -1, 1> VectorT;

    typedef spc::PointCloudBase CloudT;
    typedef CloudT::Ptr CloudPtrT;

    typedef spcSharedPtrMacro<AttitudeEstimator> Ptr;
    typedef spcSharedPtrMacro<const AttitudeEstimator> ConstPtr;

    /// def const
    AttitudeEstimator(): model_(new StratigraphicModelSingleAttitude)
    {
    }

    ///
    /// \brief getEstimatedAttitude return the current attitude as estimated by
    /// the estimator
    /// \return an spcAttitude object
    /// \note that the positioning of this attitude is centered at origin
    /// please use getEstimatedAttitudes() for having an attitude for each input
    /// cloud.
    ///
    Attitude getEstimatedAttitude(bool center = false)
    {
        Attitude out = model_->getAttitude();
        if (center)
        {
            Eigen::Vector3f c = Eigen::Vector3f::Zero();
            for (size_t i = 0; i < centroids_.size(); ++i)
            {
                c += centroids_.at(i);
            }


            c/= centroids_.size();

            out.setPosition(c);
        }

        return out;
    }



    ///
    /// \brief getEstimatedAttitudes
    /// \return  a vector of attitudes. each centered in the centroid of its
    /// parent cloud.
    ///
    std::vector<Attitude> getEstimatedAttitudes()
    {
        std::vector<Attitude> atts;
        Attitude att = getEstimatedAttitude();
        std::cout << "att: \n" << att.getNormal() << std::endl;
        Vector3f n = att.getUnitNormal();


        for (int i = 0; i < clouds_.size(); ++i) {
            Attitude new_att(n, centroids_.at(i));
            atts.push_back(new_att);
        }

        return atts;
    }

    ///
    /// \brief getEstimatedSingleAttitudeModel
    /// \return the model correspondent to the estimated attitude/s
    ///
    StratigraphicModelSingleAttitude::Ptr getEstimatedSingleAttitudeModel()
    {
        return model_;
    }

    ///
    /// \brief addInputCloud this method require 1 or more clouds t be used for
    /// estimtion
    /// \param cloud is the input cloud
    ///
    void addInputCloud(const CloudPtrT cloud)
    {
        clouds_.push_back(cloud);
        // compute its centroid


        Vector3f centroid = cloud->getCentroid();


        DLOG(INFO) << "computed centroid " << centroid.transpose();

        centroids_.push_back(centroid);
    }

    ///
    /// \brief getAveragedSquareDeviations
    /// \return  the current average squared deviation
    ///
    float getAveragedSquareDeviations()
    {
        updateDeviations(); // be sure are updated
        float acc = 0;
        int counter = 0;
        for(VectorT devs: sq_deviations_)
        {
            acc += devs.array().sum();
            counter += devs.size();
        }
        acc /= counter; // the avg
        return acc;
    }

    /// for each input cloud get its own SP as estimated with this method
    VectorT getStratigraphicPositionsOfClouds()
    {
        return est_s_positions_;
    }

    /// call the actual estimation run
    int estimate()
    {
        initializeModel();

        if (clouds_.size() == 1)
        {
            LOG(INFO) << "Only one cloud used. Not performing nonlinear optimization.";
            return 1;
        }

        VectorXf start = model_->getAttitude().getUnitNormal();

        my_functor Functor(this);

        std::cout << "BEFORE: " << start(0) << " " << start(1) << " " << start(2)
                  << std::endl;
        Eigen::NumericalDiff<my_functor> numDiff(Functor);
        LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);

        int info = lm.minimize(start);

        start = model_->getAttitude().getUnitNormal();
        std::cout << "AFTER: " << start(0) << " " << start(1) << " " << start(2)
                  << "with info " << info << std::endl;



        return info;
    }

protected:
    /// a usable functor based on float
    struct my_functor : Functor<float>
    {
        my_functor(AttitudeEstimator *est) : Functor<float>(3, 3)
        {
            estimator_ = est;
        }

        int operator()(const VectorXf &x, VectorXf &fvec) const
        {
            estimator_->model_->setNormal(x);
            float err = estimator_->getAveragedSquareDeviations();

            fvec(0) = err;
            fvec(1) = 0.0f;
            fvec(2) = 0.0f;

            DLOG(INFO)<< "CALLED functor: " << err;

            return 0;
        }

        /// a pointer to the estimator itself
        AttitudeEstimator *estimator_;
    };

    ///
    /// \brief initializeModel produce an initial estimate of the model via
    /// linear system solving
    /// it averages the normals of the best fitting planes for each input cloud
    ///
    void initializeModel()
    {
        // for each cloud we estimate a model. then we average
        // clearly only for the clouds that have at least 3 points

        std::vector<Vector3f> normals;

        for(auto & elem: clouds_)
        {
            CloudPtrT cloud = elem;
            if (cloud->getNumberOfPoints() < 3)
                continue;
            else {
                Vector3D n;
                n.normalFromBestFit(*cloud);
                normals.push_back(n.getUnitNormal());
            }
        }

        // acumualte normals
        Vector3f sum
                = std::accumulate(normals.begin(), normals.end(), Vector3f(0, 0, 0));

        sum /= normals.size(); // the average

        model_->setNormal(sum);
    }

    ///
    /// \brief updateSPs updates the stratigraphi positions for each point of
    /// the cloud
    /// usin the normal of defined in he current model
    ///
    void updateSPs()
    {
        s_positions_.clear();
        for(CloudPtrT cloud: clouds_)
        {
            VectorT vec =  spc::evaluate_dynamic_scalar_field_generator<float, size_t> (cloud, model_);

            //                model_.getScalarFieldValues(cloud);
            s_positions_.push_back(vec);
        }
    }

    ///
    /// \brief computeAveragedSPs: for each cloud associates an average sp
    /// deriving from the average of the SPs of the points contained by the
    /// cloud
    ///
    void computeAveragedSPs()
    {
        updateSPs(); // be sure are updated

        est_s_positions_.resize(s_positions_.size());


        for (int i  = 0; i < s_positions_.size(); ++i)
            //    for(std::vector<float> sps, s_positions_)
        {
            VectorT sps = s_positions_.at(i);

            float avg_sp = sps.array().sum();
            avg_sp /= sps.size();
            est_s_positions_(i) = avg_sp;
        }
    }

    ///
    /// \brief updateDeviations compute the difference between model and
    /// points - deviations are squared (sq_deviations_)
    ///
    void updateDeviations()
    {
        computeAveragedSPs(); // be sure are updated
        sq_deviations_.clear();

        for (int i = 0; i < clouds_.size(); ++i) {
            VectorT devs(s_positions_.at(i).size());

            CloudPtrT cloud = clouds_.at(i);

            float est_sp = est_s_positions_(i); // its sp
            for (int j = 0 ; j < s_positions_.at(i).size(); ++j)
            {
                float this_sp = s_positions_.at(i)(j);
                float diff = est_sp - this_sp;
                devs(j) = diff * diff;
            }

            sq_deviations_.push_back(devs);
        }
    }

    /// the input
    std::vector<CloudPtrT> clouds_;

    /// the current model
    StratigraphicModelSingleAttitude::Ptr model_;

    /// stratigraphic position for each point into the clouds
    std::vector<VectorT> s_positions_;

    /// the sp for each cloud, estimted from the model
    VectorT est_s_positions_;

    /// model to observation distance for each point into  the clouds
    std::vector<VectorT> sq_deviations_;

    /// a vector with the centroids of each cloud
    std::vector<Vector3f> centroids_;
};
}

#endif // SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H
