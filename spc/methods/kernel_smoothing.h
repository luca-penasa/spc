#ifndef SPC_KERNEL_SMOOTHING_H
#define SPC_KERNEL_SMOOTHING_H

#include <iostream>
#include <flann/flann.hpp>
#include <vector>

#include "spc/common/std_helpers.hpp"
#include <cmath>
#include <spc/time_series/equally_spaced_time_series.h>

#define _USE_MATH_DEFINES //For using PI

namespace spc
{


template <typename ScalarT>
class KernelSmoothing
{
    //Used for initializing FLANN
    typedef typename flann::L2_Simple<ScalarT> distType;
    typedef typename flann::Index<distType> FLANNIndex;
    typedef typename flann::Matrix<ScalarT> FLANNMat;

    typedef spc::EquallySpacedTimeSeries<ScalarT> OutSeriesT;
    typedef spcSharedPtrMacro<OutSeriesT> OutSeriesPtrT;


    //a shorthand for vector type
    typedef typename std::vector<ScalarT> vType;

    //and for vector of indices
    typedef typename std::vector<int> idvType;




public:
    //Constructor, takes two vectors of data, same size x and y
    KernelSmoothing();

    inline OutSeriesPtrT getOutput() const {return out_series_;}

    inline void
    setX(const vType &x)
    {
        x_ = x;
        input_modified_ = true;
        n_ = x_.size();
        initKDTree();
    }

    inline void
    setY(const vType &y){ y_ = y; input_modified_ = true; }

    inline void
    setXY(const vType &x, const  vType &y) {    x_ = x; y_ = y; input_modified_ = true; n_ = x_.size(); initKDTree(); }

    inline void setInputSeries (const GenericTimeSeries<ScalarT> & series)
    {

        this->setX(series.getX());
        this->setY(series.getY());

    }

    int
    compute();

    void
    setBandwidth(const ScalarT bandwidth)
    {
        bandwidth_ = bandwidth;
    }


    void
    setStep(const ScalarT step)
    {
        step_ = step;
    }

    void
    setComputeVariance(const bool &compute_var = true)	{ compute_var_ = compute_var; }

    inline void setOutputSeries(OutSeriesPtrT out)
    {
        out_series_ = out;
    }

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

    // the step
    ScalarT step_;

    //total number of input points
    int n_;

    vType variance_;
    bool compute_var_; //would you like to compute variance?

    //Flann index
    spcSharedPtrMacro<FLANNIndex> flann_index_;

    bool use_weights_;
    vType weights_;


    OutSeriesPtrT out_series_;

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
    initKDTree();

    int
    radiusSearch(const ScalarT &position,const ScalarT &radius, idvType &ids, vType &distances);


    int
    evaluateKS(const ScalarT &position, ScalarT &value, ScalarT &var);


};




} //end namespaces
#endif
