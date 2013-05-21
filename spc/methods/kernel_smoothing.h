#ifndef SPC_KERNEL_SMOOTHING_H
#define SPC_KERNEL_SMOOTHING_H

#include <iostream>
#include <flann/flann.hpp>
#include <flann/flann.h>
#include <vector>

#include "spc/common/std_helpers.hpp"
#include <cmath>
#define _USE_MATH_DEFINES //For using PI

//#include "GenericTimeSeries.h"
#include <spc/time_series/equally_spaced_time_series.h>

namespace spc
{


template <typename ScalarT>
class KernelSmoothing
{
    //Used for initializing FLANN
    typedef typename flann::L2_Simple<ScalarT> distType;
    typedef typename flann::Index<distType> FLANNIndex;
    typedef typename flann::Matrix<ScalarT> FLANNMat;


    //a shorthand for vector type
    typedef typename std::vector<ScalarT> vType;

    //and for vector of indices
    typedef typename std::vector<int> idvType;




public:
    //Constructor, takes two vectors of data, same size x and y
    KernelSmoothing();

    void
    setX(const vType &x){    x_ = x; input_modified_ = true;
                             n_ = x_.size();
                                                  initKDTree(); }

    void
    setY(const vType &y){ y_ = y; input_modified_ = true; }

    void
    setXY(const vType &x, const  vType &y) {    x_ = x; y_ = y; input_modified_ = true; n_ = x_.size(); initKDTree(); }

    void setInput (GenericTimeSeries<ScalarT> * series) { this->setX(series->getX()); this->setY(series->getY()); }

//    auto getOutput() -> EquallySpacedTimeSeries<ScalarT>;

    int
    compute(GenericTimeSeries<ScalarT> * new_series);


//    void
//    setEvaluationPositions(const vType &new_x)
//    {
//        new_x_ = new_x;
//        new_y_.resize(new_x_.size());
//        new_y_nans_.resize(new_x_.size());
//        variance_.resize(new_x_.size());

//    }

    void
    setBandwidth(const ScalarT &bandwidth) { bandwidth_ = bandwidth;}

    void
    setComputeVariance(const bool &compute_var = true)	{ compute_var_ = compute_var; }

    vType
    getNewY() {	return new_y_;}

    vType
    getVar()
    {
        if (compute_var_ == false)
        {
            std::cout << "ERROR: you did not enabled variance computation at compute() time \n variance vector is void" << std::endl;
        }
        return variance_;
    }

    void
    setWeights(const vType &weights) {weights_ = weights;}

    void
    getExternalWeightsForIds(const idvType ids, vType & weights);

    void
    setUseWeights(const bool useit ) {use_weights_  = useit;}

private:  //props
    //info on status
    bool input_modified_;

    //input vectors of data to be smoothed.
    vType x_;
    vType y_;

    //output
    vType new_x_;
    vType new_y_;
    //A vector where to keep track of nan in output
    std::vector<bool> new_y_nans_;

    //bandwidth
    ScalarT bandwidth_;

    //total number of input points
    int n_;

    vType variance_;
    bool compute_var_; //would you like to compute variance?

    //Flann index
    FLANNIndex* flann_index_;

    bool use_weights_;
    vType weights_;

private: //methods
    //compute gaussian weights on a vector
    void
    gaussian(const vType &values, vType &gaussian_values);


    //compute gaussian weights on a single numeric value
    inline ScalarT
    gaussian(const ScalarT &value)
    {
        return 1.0/sqrt(2.0*M_PI) * exp(-0.5*value*value);
    }

    void
    initKDTree()
    {
        flann_index_ = new FLANNIndex (flann::Matrix<ScalarT> (&x_[0], n_, 1), flann::KDTreeSingleIndexParams (15));
        flann_index_->buildIndex();

    }

    int
    radiusSearch(const ScalarT &position,const ScalarT &radius, idvType &ids, vType &distances);


    int
    evaluateKS(const ScalarT &position, ScalarT &value, ScalarT &var);


};




} //end namespaces
#endif
