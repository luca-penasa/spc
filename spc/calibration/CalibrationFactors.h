#ifndef CALIBRATIONFACTORS_H
#define CALIBRATIONFACTORS_H
//#include "ceres/ceres.h"
//#include "glog/logging.h"

#include <spc/elements/ElementBase.h>
#include <spc/calibration/Observations.h>
#include <spc/calibration/SampledData.h>
namespace spc {



//class BasicFactor
//{
//public:

//    virtual size_t getNumberOfBlocks() const = 0;
//    virtual size_t getFirstBlockIndex() const = 0;
//    virtual size_t getNumberOfParametersInBlock(const size_t & block_id) const = 0;

//    template <typename T>
//    std::vector< Eigen::Map<Eigen::Matrix<T, -1, 1> > >
//    parseParameters(T const * const * parameters)
//    {
//        size_t n_blocks =  this->getNumberOfBlocks();
//        std::vector< Eigen::Map<Eigen::Matrix<T, -1, 1>>> out(n_blocks);

//        for (int i = 0 ; i < n_blocks; ++i)
//        {
//            size_t n_pars_in_block = this->getNumberOfParametersInBlock(i);

//            T const * myblock = parameters[this->getFirstBlockIndex()];

//            out.at(i) = Eigen::Map<Eigen::Matrix<T, -1, 1>> (myblock, n_pars_in_block);
//        }

//        return out;
//    }

//};

//class PerCloudFactor : public BasicFactor
//{
//public:

//    PerCloudFactor(Eigen::VectorXi cloud_ids, int fixed_id = 0)
//    {
//        fixed_id_ = fixed_id;
//        n_ids_ = cloud_ids.rows();

//        size_t index = 0;

//        for (int i = 0 ; i < n_ids_; ++i)
//        {
//            int cloud_id = cloud_ids(i);

//            if (cloud_id != fixed_id)
//            {
//                cloud_id_to_coeff_id_.at(cloud_id) = index++;
//            }
//        }

//        coefficients_ = Eigen::VectorXd (index);
//    }

//    template <typename T> T value(const Observation &obs,
//                                  const std::vector<Eigen::Map<Eigen::Matrix<T, -1, 1>>> &blocks)
//    {
//        //!TODO we could check that the blocks are of the right type
//        int cloud_id = obs.cloud_id;
//        size_t index = cloud_id_to_coeff_id_.at(cloud_id);
//        return blocks.at(0)(index);
//    }



//    std::map<int, size_t> cloud_id_to_coeff_id_;

//    Eigen::Matrix<double, -1,1> coefficients_;

//    int fixed_id_;
//    int n_ids_;

//    // BasicFactor interface
//public:
//    virtual size_t getNumberOfBlocks() const
//    {
//        return 1;
//    }

//    virtual size_t getFirstBlockIndex() const
//    {
//        return 0;
//    }


//    virtual size_t getNumberOfParametersInBlock(const size_t &block_id) const
//    {
//        return coefficients_.rows();
//    }

//};

//template<typename T>
//T getPerCloudCoefficients(const Observation &ob,const PerCloudFactor &factors, T const * const * parameters)
//{
//    std::vector<Eigen::Map<Eigen::Matrix<T, -1, 1>>> blocks = factors.parseParameters(parameters);


//}

}//end nspace
#endif // CALIBRATIONFACTORS_H
