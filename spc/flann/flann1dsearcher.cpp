#include "flann1dsearcher.h"
#include <flann/flann.hpp>


namespace spc {



template<typename ScalarT>
Flann1DSearcher<ScalarT>::Flann1DSearcher(std::vector<ScalarT> v)
{
    v_ = v;

    FLANNMat mat = FLANNMat (v_.data(), v_.size(), 1);

    flann::KDTreeSingleIndexParams pars(15);
    flann_index_.reset(new FLANNIndex (mat, pars));
    flann_index_->buildIndex();
}


template class Flann1DSearcher<float>;
template class Flann1DSearcher<double>;


}
