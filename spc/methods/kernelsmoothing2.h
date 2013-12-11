#ifndef KERNELSMOOTHING2_H
#define KERNELSMOOTHING2_H

#include <spc/time_series/sparse_time_series.h>
#include <spc/time_series/equally_spaced_time_series.h>


#include <flann/flann.hpp>

#include <pcl/console/print.h>

namespace spc
{

template <typename ScalarT>
class KernelSmoothing2
{

public:

    typedef SparseTimeSeries<ScalarT> SparseT;
    typedef boost::shared_ptr<SparseT> SparsePtrT;

    typedef EquallySpacedTimeSeries<ScalarT> EquallyT;
    typedef boost::shared_ptr<EquallyT> EquallyPtrT;



    typedef typename flann::L2_Simple<ScalarT> distType;
    typedef typename flann::Index<distType> FLANNIndex;
    typedef typename flann::Matrix<ScalarT> FLANNMat;


    KernelSmoothing2(): step_(1.0), bandwidth_(1.0)
    {

    }

    void setInputSeries(SparsePtrT in)
    {
        sparse_ = in;
    }

    void setOutputSeriesBlank(EquallyPtrT out)
    {
        out_series_ = out;
    }


    void setStep(ScalarT step)
    {
        step_ = step;
    }

    void setBandwidth(ScalarT bandwidth)
    {
        bandwidth_ = bandwidth;
    }


    EquallyPtrT getOutputSeries() const
    {
        return out_series_;
    }

    int compute()
    {
        if (!out_series_)
        {
            out_series_ = EquallyPtrT(new EquallyT(sparse_->getMinX(), sparse_->getMaxX(), step_));
            pcl::console::print_warn("You could fix the sizes of out series setting the output time series\n");
        }



        FLANNMat mat = FLANNMat (&sparse_->getX()[0], sparse_->getNumberOfSamples(), 1);
        flann::KDTreeSingleIndexParams pars  = flann::KDTreeSingleIndexParams(15);
        flann_index_.reset(new FLANNIndex (mat, pars));
        flann_index_->buildIndex();

        ScalarT value;

#ifdef DUSE_OPENMP
#pragma omp parallel for private (value)
#endif

        int count = 0;
        for (auto x: out_series_->getX())
        {
            evaluateKS(x, value);
            out_series_->getY(count++) = value;
        }




        return 1;
    }

protected:
    SparsePtrT sparse_;
    EquallyPtrT out_series_;

    ScalarT bandwidth_;
    ScalarT step_;

    boost::shared_ptr<FLANNIndex> flann_index_;

    //compute gaussian weights on a vector
    inline void
    gaussian(const std::vector<ScalarT> &values, std::vector<ScalarT> &gaussian_values)
    {
        for (auto val: values)
            gaussian_values.push_back(gaussian(val));
    }


    //compute gaussian weights on a single numeric value
    inline ScalarT
    gaussian(const ScalarT &value)
    {
        return 1.0/sqrt(2.0*M_PI) * exp(-0.5*value*value);
    }

    void
    initKDTree();

    int
    radiusSearch(const ScalarT &position,const ScalarT &radius, std::vector<int> &ids, std::vector<ScalarT> &distances)
    {
        bool sorted = true; //we are not interested in sorting depending on distances
        ScalarT epsilon = 0.0; // we want exact results
        ScalarT radius2 = radius*radius; //FLANN works with squared radius

        //put position in a vector so to be passed to flann
        std::vector<ScalarT> pos(1); //just a search for time is supported
        pos[0] = position;

        //for storing results
        flann::Matrix<int> ids_empty;
        flann::Matrix<ScalarT> d_empty;

        //initialize search parameters
        flann::SearchParams search_params = flann::SearchParams (-1 ,epsilon, sorted);

        //now search
        int nn = flann_index_->radiusSearch(FLANNMat(&pos[0], 1, 1), ids_empty, d_empty, radius2, search_params);


        //resize the output vectors to the number of found neighbors
        ids.resize(nn);
        distances.resize(nn);

        //now get the output vectors as flann::Matrix
        flann::Matrix<int> ids_mat (&ids[0], 1, nn);
        flann::Matrix<ScalarT> distances_mat (&distances[0], 1, nn);

        //redo search and write output
        flann_index_->radiusSearch(FLANNMat(&pos[0], 1, 1), ids_mat , distances_mat, radius2, search_params);

        for (auto &d: distances)
            d = sqrt(d);


        return nn;
    }

    int
    evaluateKS(const ScalarT &position, ScalarT &value)
    {
        //Note that gaussian kernel is not compactly supported,
        //we restrict the neighbors extraction to a compact region on x
        //say 4 times the bandwidth

        ScalarT support_region = bandwidth_ * 4;

        //now get all points inside this support region
        std::vector<int> ids;
        std::vector<ScalarT> distances;
        int nn = radiusSearch(position, support_region, ids, distances);

        if (nn == 0)
        {
            pcl::console::print_debug("No neighbors found -> set to NaN");
            value = std::numeric_limits<ScalarT>::quiet_NaN ();
            return 1;
        }


        for (auto &d: distances)
            d = d/bandwidth_;

        //now compute the weights
        std::vector<ScalarT> weights;
        gaussian(distances, weights);

        std::vector<ScalarT> vproduct; //element by element product TODO this should be done as above

        vproduct.resize(ids.size());
        for (int i = 0; i < ids.size(); ++i)
        {
            int this_id = ids.at(i);
            vproduct.at(i) = weights.at(i) * this->sparse_->getY().at(this_id);
        }

        //sum this product
        ScalarT vprod_sum = std::accumulate(vproduct.begin(),vproduct.end(), 0.0);

        //and sum weights
        ScalarT weights_sum  = std::accumulate(weights.begin(), weights.end(), 0.0);

        //and get the actual average
        value = vprod_sum / weights_sum;


        //        //now compute weighted variance
        //        if (compute_var_ == true)
        //        {
        //            vType difference;
        //            difference.resize(nn);
        //            for (int i = 0; i < nn; ++i)
        //            {
        //                int this_id = ids[i];
        //                difference[i] = y_[this_id] - value;

        //            }
        //            vType squared = get_squared(difference);
        //            vType product = get_product(difference, weights);


        //            ScalarT sum1 = get_sum(product);
        //            var = sum1 / ((nn-1)* weights_sum / nn);
        //        }
        //        else
        //        {
        //            var = 0.0;
        //        }

        return 1;

    }





};

}//end nspace

#endif // KERNELSMOOTHING2_H
