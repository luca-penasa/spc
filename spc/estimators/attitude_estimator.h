#ifndef SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H
#define SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H

#include <spc/stratigraphy/single_attitude_model.h>


#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

#include <iostream>

namespace spc
{


/// a templated version of the generic functor to be used in nonlinear optimizations
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

};



///
/// \brief The SinglePlaneModelFromMultiCloudEstimator class
///
class AttitudeEstimator
{
public:

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> CloudT;
    typedef CloudT::Ptr CloudPtrT;

    typedef typename boost::shared_ptr<AttitudeEstimator> Ptr;

    ///def const
    AttitudeEstimator();

    ///
    /// \brief getEstimatedAttitude return the current attitude as estimated by the estimator
    /// \return an Attitude object
    /// \note that the positioning of this attitude is centered at origin
    /// please use getEstimatedAttitudes() for having an attitude for each input cloud.
    ///
    Attitude getEstimatedAttitude() {return static_cast<Attitude> (model_);}

    ///
    /// \brief getEstimatedAttitudes
    /// \return  a vector of attitudes. each centered in the centroid of its parent cloud.
    ///
    std::vector<Attitude> getEstimatedAttitudes();

    ///
    /// \brief getEstimatedSingleAttitudeModel
    /// \return the model correspondent to the estimated attitude/s
    ///
    SingleAttitudeModel getEstimatedSingleAttitudeModel() {return model_;}


    ///
    /// \brief addInputCloud this method require 1 or more clouds t be used for estimtion
    /// \param cloud is the input cloud
    ///
    void addInputCloud(CloudPtrT cloud);

    ///
    /// \brief getAveragedSquareDeviations
    /// \return  the current average squared deviation
    ///
    float getAveragedSquareDeviations();

    /// for each input cloud get its own SP as estimated with this method
    std::vector<float> getStratigraphicPositionsOfClouds();

    /// call the actual estimation run
    int estimate();

private:
    /// a usable functor based on float
    struct my_functor : Functor<float>
    {
        my_functor(AttitudeEstimator * est): Functor<float>(3,3)
        {
            estimator_ = est;
        }

        int operator()(const VectorXf &x, VectorXf &fvec) const
        {
            estimator_->model_.setNormal(x);
            float err = estimator_->getAveragedSquareDeviations();

            fvec(0) = err;
            fvec(1) = 0.0f;
            fvec(2) = 0.0f;

            std::cout << "CALLED functor: " << err << std::endl;

            return 0;
        }

        /// a pointer to the estimator itself
        AttitudeEstimator * estimator_;
    };




    ///
    /// \brief initializeModel produce an initial estimate of the model via linear system solving
    /// it averages the normals of the best fitting planes for each input cloud
    ///
    void initializeModel();

    ///
    /// \brief updateSPs updates the stratigraphi positions for each point of the cloud
    /// usin the normal of defined in he current model
    ///
    void updateSPs();

    ///
    /// \brief computeAveragedSPs: for each cloud associates an average sp
    /// deriving from the average of the SPs of the points contained by the cloud
    ///
    void computeAveragedSPs();

    ///
    /// \brief updateDeviations compute the difference between model and
    /// points - deviations are squared (sq_deviations_)
    ///
    void updateDeviations();

    /// the input
    std::vector<CloudPtrT> clouds_;

    /// the current model
    SingleAttitudeModel model_;   

    /// stratigraphic position for each point into the clouds
    std::vector<std::vector<float> > s_positions_;

    /// the sp for each cloud, estimted from the model
    std::vector<float> est_s_positions_;

    /// model to observation distance for each point into  the clouds
    std::vector<std::vector<float> > sq_deviations_;

    /// a vector with the centroids of each cloud
    std::vector<Vector3f> centroids_;
};


}

#endif // SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H
