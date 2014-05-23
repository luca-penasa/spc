#ifndef SPC_FLANN1DSEARCHER_H
#define SPC_FLANN1DSEARCHER_H

#include <flann/flann.hpp>
#include <spc/common/macros.h>
namespace spc {



template <typename ScalarT>
class Flann1DSearcher
{
public:
    SPC_OBJECT(Flann1DSearcher)

    Flann1DSearcher(std::vector<ScalarT> v);

    typedef typename flann::L2_Simple<ScalarT> distType;
    typedef typename flann::Index<distType> FLANNIndex;
    typedef typename flann::Matrix<ScalarT> FLANNMat;


protected:
    // the "searchable" vector
    std::vector<ScalarT> v_;

    spcSharedPtrMacro<FLANNIndex> flann_index_;

};



} //end nspace

#endif // SPC_FLANN1DSEARCHER_H
