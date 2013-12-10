#include <spc/methods/kernel_smoothing.h>
#include <spc/common/std_helpers.hpp>


namespace spc
{



template<typename ScalarT>
KernelSmoothing<ScalarT>::KernelSmoothing() : weights_(0), step_(1.0), bandwidth_(1.0)
{

    //defaults
    compute_var_ = false;    
    input_modified_ = true;
    use_weights_ = false;

}


template<typename ScalarT>
int
KernelSmoothing<ScalarT>::compute()
{

    if (!out_series_)
    {
        out_series_ = OutSeriesPtrT(new OutSeriesT(get_min(x_), get_max(x_), step_));
        std::cout << "time series wil be automaticcaly of full size" << std::endl;
    }

    new_x_ = out_series_->getX();
    new_y_.resize(new_x_.size());

    new_y_nans_.resize(new_x_.size());
    variance_.resize(new_x_.size());

    ScalarT value;
    ScalarT variance;
#ifdef DUSE_OPENMP
    #pragma omp parallel for private (value, variance)
#endif
    for (int i = 0; i < new_y_.size(); ++i)
    {

        evaluateKS(new_x_.at(i), value, variance);

        if (!std::isnan(value))
            new_y_nans_[i] = false;
        else
            new_y_nans_[i] = true;

        new_y_[i] = value;
        if (compute_var_ == true)
            variance_[i] = variance;
    }

    out_series_->setY(new_y_);

    return 1;
}





template<typename ScalarT>
void
KernelSmoothing<ScalarT>::gaussian(const vType &values, vType &gaussian_values)
{

    gaussian_values.resize(values.size());
    for (int i = 0; i < values.size(); ++i)
    {
        gaussian_values[i] = gaussian(values[i]);
    }

}

template<typename ScalarT>
int 
KernelSmoothing<ScalarT>::radiusSearch(const ScalarT &position, const ScalarT &radius, idvType &ids, vType &distances)
{
    bool sorted = true; //we are not interested in sorting depending on distances
    ScalarT epsilon = 0.0; // we want exact results
    ScalarT radius2 = radius*radius; //FLANN works with squared radius

    //put position in a vector so to be passed to flann
    vType pos(1); //just a search for time is supported
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

    //getting sqrt of distances
    for (int i = 0; i < nn; ++i)
    {
        distances[i] = sqrt(distances[i]);

    }
    return nn;
}

template<typename ScalarT>
int
KernelSmoothing<ScalarT>::evaluateKS(const ScalarT &position, ScalarT &value, ScalarT &var)
{
    //Note that gaussian kernel is not compactly supported,
    //we restrict the neighbors extraction to a compact region on x
    //say 2 times the bandwidth

    ScalarT support_region = bandwidth_ * 2;

    //now get all points inside this support region
    idvType ids;
    vType distances;
    int nn = radiusSearch(position, support_region, ids, distances);

    if (nn == 0)
    {
        value = std::numeric_limits<ScalarT>::quiet_NaN ();
        var = std::numeric_limits<ScalarT>::quiet_NaN ();
        return 1;
    }

    //divide the distances for the bandwidth
    for (int i = 0; i < ids.size(); ++i )
    {
        distances[i] = distances[i] / bandwidth_;
    }

    //now compute the weights
    vType weights;
    gaussian(distances, weights);



    //if it is the case also the external weights
    if (use_weights_)
    {

        vType external_w;
        getExternalWeightsForIds(ids, external_w);
        //multiply with the gaussian based weights!
        //NOTE is in place, hope it works
        std::transform(weights.begin(), weights.end(), external_w.begin(), weights.begin(), std::multiplies<ScalarT>() );
    }

    vType vproduct; //element by element product TODO this should be done as above
    vproduct.resize(ids.size());
    for (int i = 0; i < ids.size(); ++i)
    {
        int this_id = ids[i];
        vproduct[i] = weights[i] * this->y_.at(this_id);
    }

    //sum this product
    ScalarT vprod_sum = 0;
    for (int i = 0; i < ids.size(); ++i)
    {
        vprod_sum += vproduct[i];
    }

    //and sum weights
    ScalarT weights_sum = 0;
    for (int i = 0; i < ids.size(); ++i)
    {
        weights_sum += weights[i];
    }

    value = vprod_sum / weights_sum;


    //now compute weighted variance
    if (compute_var_ == true)
    {
        vType difference;
        difference.resize(nn);
        for (int i = 0; i < nn; ++i)
        {
            int this_id = ids[i];
            difference[i] = y_[this_id] - value;

        }
        vType squared = get_squared(difference);
        vType product = get_product(difference, weights);


        ScalarT sum1 = get_sum(product);
        var = sum1 / ((nn-1)* weights_sum / nn);
    }
    else
    {
        var = 0.0;
    }

    return 1;

}

template <typename ScalarT>
void
KernelSmoothing<ScalarT>::getExternalWeightsForIds(const idvType ids, vType &weights)
{
    weights.resize(ids.size());

    size_t counter = 0;
    for (auto id: ids)
        weights.at(counter++) = this->weights_.at(id);
}

template<typename ScalarT>
void KernelSmoothing<ScalarT>::initKDTree()
{
    FLANNMat mat = FLANNMat (&x_[0], n_, 1);
    flann::KDTreeSingleIndexParams pars  = flann::KDTreeSingleIndexParams(15);
    flann_index_.reset(new FLANNIndex (mat, pars));
    flann_index_->buildIndex();
}


////////// INSTANTIATIONS

template class KernelSmoothing<float>;
template class KernelSmoothing<double>;


} //closing namespaces
