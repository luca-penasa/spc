#include "KernelSmoothing2.h"
#include <numeric>


namespace spc
{



KernelSmoothing2::KernelSmoothing2()


{
}

int KernelSmoothing2::compute()
{
    DLOG(INFO) << "starting computation of kernel smoothing";

    if (!out_series_) {
        out_series_ = EquallyPtrT(
            new EquallyT(sparse_->getMinX(), sparse_->getMaxX(), step_));
        pcl::console::print_warn("You could fix the sizes of out series "
                                 "setting the output time series\n");
    }

    extractVectors();

    initFlann();

    DLOG(INFO) << "flann should be init";
    DCHECK (nanoflann_index_ != NULL);

    ScalarT value;

//    #ifdef USE_OPENMP
//        #pragma omp parallel for private (value)
//    #endif

    int count = 0;

    std::vector<ScalarT> x_val;
    out_series_->getX(x_val);
    spcForEachMacro(ScalarT x, x_val)
    {
        evaluateKS(x, value);
        out_series_->getY(count++) = value;
    }

    LOG(INFO) << "computation done";

    return 1;
}

void KernelSmoothing2::initFlann()
{
    DLOG(INFO) << "Going to init a nanoflann index";
//get our data as an eigen matrix
    Eigen::Map<Eigen::MatrixXf> data = Eigen::Map<Eigen::MatrixXf> (x_.data(), x_.size(), 1);

    Eigen::MatrixXf real_mat = data;



    DLOG(INFO) << "DATA MATRIX \n" << data;
    nanoflann_index_ = spcSharedPtrMacro<NanoFlannIndexT> (new NanoFlannIndexT(1, real_mat, 10));

    DLOG(INFO) << "going to call build index";
    nanoflann_index_->index->buildIndex();

    DLOG(INFO) << "Going to init a nanoflann index. Done.";
}

void KernelSmoothing2::extractVectors()
{

    DLOG(INFO)<< "extracting vectors";

    x_.clear();
    y_.clear();

    std::vector<ScalarT> x, y;
    sparse_->getX(x);
    sparse_->getY(y);

    /// remove nan values both if their are in x or y series
    for (int i = 0; i < x.size(); ++i) {
        if ((std::isfinite(x.at(i))) && (std::isfinite(y.at(i)))) {
            x_.push_back(x.at(i));
            y_.push_back(y.at(i));
        }
    }

    DLOG(INFO) <<"extracting vectors. Done";
}

int KernelSmoothing2::radiusSearch(const ScalarT &position,
                                   const ScalarT &radius,
                                   std::vector<std::pair<size_t,ScalarT> > &matches)
{

//    LOG(INFO) << "call radiussearch";

    std::vector<std::pair<size_t,ScalarT> > ret_matches;
    nanoflann::SearchParams params;

    const size_t nMatches = nanoflann_index_->index->radiusSearch(&position, radius*radius, matches, params);




//    LOG_EVERY_N(INFO, 1) << nMatches;


    return nMatches;
}

int KernelSmoothing2::evaluateKS(const ScalarT &position, ScalarT &value)
{
    // Note that gaussian kernel is not compactly supported,
    // we restrict the neighbors extraction to a compact region on x
    // say 4 times the bandwidth

    ScalarT support_region = bandwidth_ * 4;

    // now get all points inside this support region
//    std::vector<int> ids;
    std::vector<std::pair<size_t, ScalarT>> matches;
    int nn = radiusSearch(position, support_region, matches);


    if (nn == 0) {
        pcl::console::print_debug("No neighbors found -> set to NaN");
        value = std::numeric_limits<ScalarT>::quiet_NaN();
        return 1;
    }


    typedef std::pair<size_t, ScalarT> MatchT;

    // scale the distance
    spcForEachMacro(MatchT & match, matches)
    {
        // the suqared distance / the bandwidth
        match.second = gaussian(match.second * match.second / bandwidth_);
    }



    std::vector<ScalarT> vproduct; // element by element product TODO this
                                   // should be done as above


    float wsum = 0;
    for (const auto match: matches)
    {
        vproduct.push_back(match.second * y_.at(match.first));
        wsum += match.second;
    }

    // sum this product
    ScalarT vprod_sum = std::accumulate(vproduct.begin(), vproduct.end(), 0.0);

    // and get the actual average
    value = vprod_sum / wsum;

    return 1;
}

} // end nspace
