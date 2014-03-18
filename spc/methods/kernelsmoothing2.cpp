#include "kernelsmoothing2.h"
#include <numeric>
namespace spc
{



template <typename ScalarT>
KernelSmoothing2<ScalarT>::KernelSmoothing2(): step_(1.0), bandwidth_(1.0)
{
    pars_["leaf_max_size"] = 15;
}



template <typename ScalarT>
int KernelSmoothing2<ScalarT>::compute()
{

    std::cout << "started computing 0" << std::endl;

    if (!out_series_)
    {
        out_series_ = EquallyPtrT(new EquallyT(sparse_->getMinX(), sparse_->getMaxX(), step_));
        pcl::console::print_warn("You could fix the sizes of out series setting the output time series\n");
    }

    std::cout << "started computing 1" << std::endl;


    initFlann();

    if (!flann_index_)
    {
        pcl::console::print_error("[Error] some error in flann computation and initialization." );
    }
    ScalarT value;


    std::cout << "started computing 2 " << flann_index_ << std::endl;


//#ifdef USE_OPENMP
//    #pragma omp parallel for private (value)
//#endif

    int count = 0;

    std::vector<ScalarT> x_val = out_series_->getX();
    BOOST_FOREACH( ScalarT x, x_val)
    {
        evaluateKS(x, value);
        out_series_->getY(count++) = value;
    }




    return 1;
}

template<typename ScalarT>
void KernelSmoothing2<ScalarT>::initFlann()
{

    if (!flann_index_)
    {
        FLANNMat mat = FLANNMat (&sparse_->getX()[0], sparse_->getNumberOfSamples(), 1);
        std::cout << "crash here" << std::endl;

//        flann::IndexParams pars_test;
//        pars["leaf_max_size"] = 15;

                std::cout << "crash here 2" << std::endl;

        flann_index_.reset(new FLANNIndex (mat, pars_));
        flann_index_->buildIndex();
    }
}


template<typename ScalarT>
int KernelSmoothing2<ScalarT>::radiusSearch(const ScalarT &position, const ScalarT &radius, std::vector<int> &ids, std::vector<ScalarT> &distances)
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
    SearchParams search_params = SearchParams (-1 ,epsilon, sorted);

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

    int counter = 0 ;
    BOOST_FOREACH (const float d, distances)
    {
       distances.at(counter++) = sqrt(d);
    }

    return nn;
}


template<typename ScalarT>
int KernelSmoothing2<ScalarT>::evaluateKS(const ScalarT &position, ScalarT &value)
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

    int counter = 0 ;
    BOOST_FOREACH(const float d, distances)
    {
        distances.at(counter++) = d/bandwidth_;
    }
    //now compute the weights
    std::vector<ScalarT> weights;
    gaussian(distances, weights);

    std::vector<ScalarT> vproduct; //element by element product TODO this should be done as above

    vproduct.resize(ids.size());
    std::vector<ScalarT> y_vals = sparse_->getY();

    for (int i = 0; i < ids.size(); ++i)
    {
        int this_id = ids.at(i);
        vproduct.at(i) = weights.at(i) * y_vals.at(this_id);
    }

    //sum this product
    ScalarT vprod_sum = std::accumulate(vproduct.begin(),vproduct.end(), 0.0);

    //and sum weights
    ScalarT weights_sum  = std::accumulate(weights.begin(), weights.end(), 0.0);

    //and get the actual average
    value = vprod_sum / weights_sum;


    return 1;

}

template class KernelSmoothing2<float>;
template class KernelSmoothing2<double>;


} // end nspace
