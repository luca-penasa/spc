#ifndef SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H
#define SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H

#include <spc/geology/normal_estimator_base.h>
#include <spc/geology/single_plane_stratigraphic_model.h>
#include <spc/geology/single_plane_model_from_one_cloud_estimator.h>

#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

#include <iostream>

namespace spc
{

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

class SinglePlaneModelFromMultiCloudEstimator : public NormalEstimatorBase
{
public:

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> CloudT;
    typedef CloudT::Ptr CloudPtrT;

    typedef typename boost::shared_ptr<SinglePlaneModelFromMultiCloudEstimator> Ptr;

    SinglePlaneModelFromMultiCloudEstimator();

private:
    struct my_functor : Functor<float>
    {
        my_functor(SinglePlaneModelFromMultiCloudEstimator * est): Functor<float>(3,3)
        {
            estimator_ = est;
        }

        int operator()(const VectorXf &x, VectorXf &fvec) const
        {
            estimator_->model_.setUnitNormal(x);
            float err = estimator_->getAveragedSquareDeviations();

            fvec(0) = err;
            fvec(1) = 0.0f;
            fvec(2) = 0.0f;

            std::cout << "CALLED functor: " << err << std::endl;

            return 0;
        }

        SinglePlaneModelFromMultiCloudEstimator * estimator_;
    };





    void initializeModel()
    {
        //for each cloud we estimate a model. then we average
        //clearly only for the clouds that have at least 3 points

        int counter = 0;
        std::vector<Vector3f> normals;

        for (int i = 0; i < clouds_.size(); ++i)
        {
            CloudPtrT cloud = clouds_.at(i);
            if (cloud->size() < 3)
                continue;
            else
            {
                SinglePlaneStratigraphicModel::Ptr model (new SinglePlaneStratigraphicModel);
                SinglePlaneModelFromOneCloudEstimator estimator;
                estimator.setInputCloud(cloud);
                estimator.estimate(*model); //we are assuming we are able to estimate something!

                normals.push_back(model->getUnitNormal());
            }
        }


        //acumualte normals
        Vector3f sum = std::accumulate(normals.begin(), normals.end(), Vector3f (0,0,0));

        sum /= normals.size(); //the average

        model_.setUnitNormal(sum); //setunitnormal automatically normalize the vector
    }

    void updateSPs()
    {
        s_positions_.clear();
        for (CloudPtrT cloud: clouds_)
        {
            auto vec = model_.getStratigraphicPositions(cloud);
            s_positions_.push_back(vec);
        }
    }

    ///
    /// \brief computeAveragedSPs: for each cloud associates an average sp
    ///
    void computeAveragedSPs()
    {
        updateSPs(); //be sure are updated

        est_s_positions_.clear(); //clear it

        for (std::vector<float> sps: s_positions_)
        {
            float avg_sp = std::accumulate(sps.begin(), sps.end(), 0.0);
            avg_sp /= sps.size();
            est_s_positions_.push_back(avg_sp);
        }
    }

    void updateDeviations()
    {
        computeAveragedSPs(); //be sure are updated
        sq_deviations_.clear();

        for (int i = 0; i < clouds_.size(); ++i)
        {
            std::vector<float> devs;
            CloudPtrT cloud = clouds_.at(i);
            float est_sp = est_s_positions_.at(i); //its sp
            for (float this_sp: s_positions_.at(i))
            {
                float diff = est_sp - this_sp;
                devs.push_back(diff*diff);
            }

            sq_deviations_.push_back(devs);

        }
    }

public:


    void addInputCloud(CloudPtrT cloud)
    {
        clouds_.push_back(cloud);
    }


    float getAveragedSquareDeviations()
    {
        updateDeviations(); //be sure are updated
        float acc = 0;
        int counter =0;
        for (std::vector<float> devs: sq_deviations_)
        {
            acc += std::accumulate(devs.begin(), devs.end(), 0.0f);
            counter += devs.size();
        }
        acc /= counter; //the avg
        return acc;
    }

    std::vector<float> getStratigraphicPositionsOfClouds()
    {
        return est_s_positions_;
    }

    int estimate(SinglePlaneStratigraphicModel &model)
    {
         initializeModel();

         VectorXf start = model_.getUnitNormal();

         my_functor Functor(this);

         std::cout << "BEFORE: "<< start(0) << " " << start(1) << " " << start(2) << std::endl;
         Eigen::NumericalDiff<my_functor> numDiff(Functor);
         LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);


         int info = lm.minimize(start);

         model.setUnitNormal( model_.getUnitNormal() );

         std::cout << "AFTER: " << start(0) << " " << start(1) << " " << start(2) << "with info " << info << std::endl;

         return info;
    }



private:
    std::vector<CloudPtrT> clouds_;

    SinglePlaneStratigraphicModel model_;

    /// stratigraphic position for each point into the clouds
    std::vector<std::vector<float> > s_positions_;

    /// the sp for each cloud, estimted from the model
    std::vector<float> est_s_positions_;

    /// model to observation distance for each point into  the clouds
    std::vector<std::vector<float> > sq_deviations_;
};


}

#endif // SINGLE_PLANE_MODEL_FROM_MULTI_CLOUD_ESTIMATOR_H
