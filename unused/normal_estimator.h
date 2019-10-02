#ifndef NORMAL_ESTIMATOR_H
#define NORMAL_ESTIMATOR_H

#include <spc/geology/stratigraphic_normal_model.h>
#include <spc/geology/normal_estimator_base.h>
#include <spc/geology/geologic_plane.h>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>

#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

#include <numeric>

namespace spc {

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

  // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};



using namespace Eigen;
///
/// \brief The NormalEstimator class permits to estimate a stratigraphic normal from sets of points
/// representing "planes" in 3d space. From one or more "planar" clouds this class
/// estimates only ONE normal, as average of the normals, or as non-linear optimization of the normal
/// using all the different planes.
///
class NormalEstimator
{
public:
    NormalEstimator();

    struct my_functor : Functor<float>
    {
        my_functor(NormalEstimator * est): Functor<float>(3,3)
        {
            estimator_ = est;
        }

        int operator()(const VectorXf &x, VectorXf &fvec) const
        {
            estimator_->model_.setNormal(x);
            estimator_->estimateStratigraphicPositionsOfPlanes();
            estimator_->computeSquaredErrors();

            float err = estimator_->getOverallSquaredError();

            fvec(0) = err;
            fvec(1) = 0.0f;
            fvec(2) = 0.0f;

            std::cout << "CALLED functor: " << err << std::endl;

            return 0;
        }

        NormalEstimator * estimator_;
    };

    void addPlanarCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud)
    {
        if (planar_cloud->size() == 0 )
        {
            std::cout << "ERR you should have at least one point for defining a stratigraphic position! Not Added!" << std::endl;
            return;
        }
        planar_clouds_.push_back(planar_cloud);
    }

    bool getStratigraphicPositionForCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &sp)
    {
        for (int i = 0 ; i < planar_clouds_.size(); ++i)
        {
            if (cloud == planar_clouds_.at(i))
            {
                sp = stratigraphic_positions_.at(i);
                return true;
            }
        }

        return false; // only if we reach the end without a valid match
    }

    ///
    /// \brief estimateNormalAsAverage permits to estimate a normal as average of a series
    /// of planes represented by clouds
    ///
    int estimateNormalAsAverage();

    int estimateNormalAsNonlinearOpt();


    //!
    //! \brief getNormalModel get the model
    //! \return returns a StratigraphicNormalModel
    //!
    StratigraphicNormalModel getNormalModel() const {return model_;}

    //!
    //! \brief getAverageRMS
    //! \return get the average RMS, it is != 0 only when the average method is used
    //!
    float getAverageRMS() const {return avg_rms_;}

    GeologicPlane::Ptr
    fromSingleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr one_plane_cloud, float &rms);



    void estimateStratigraphicPositionsOfPlanes();

    void computeSquaredErrors();


    //for the current model
    float getOverallSquaredError();

    //! set of input clouds, each is a plane in 3d
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > planar_clouds_;

    //! the model we need to estimate - actually the normal plus the strat pos.
    StratigraphicNormalModel model_;

    /////// USED IN AVERAGE MODE //////////////

    //! when the model is computed as average this is populated with a normal for each plane
    std::vector<GeologicPlane> normals_;

    //! also the RMS for each plane is here saved (in average mode)
    std::vector<float> rmss_;

    //! we compute an average of the rmss
    float avg_rms_;

    /////// USED IN MULTI-PLANE MODE //////////////

    //! Stratigraphic positions of the planes, based on the current model, use estimateStratigraphicPositionsOfPlanes() for updating
    std::vector<float> stratigraphic_positions_;

    //! errors for any plane
    std::vector<float> plane_squared_errors_;

};

} //end nspace

#endif // NORMAL_ESTIMATOR_H

