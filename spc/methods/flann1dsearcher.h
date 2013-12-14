#ifndef FLANN1DSEARCHER_H
#define FLANN1DSEARCHER_H

#include <boost/shared_ptr.hpp>
#include <flann/flann.hpp>



template <typename ScalarT>
class Flann1DSearcher
{
public:

    typedef boost::shared_ptr<Flann1DSearcher<ScalarT> > Ptr;
    typedef boost::shared_ptr<const Flann1DSearcher<ScalarT> > ConstPtr;

    Flann1DSearcher(const std::vector<ScalarT> v);

    typedef typename flann::L2_Simple<ScalarT> distType;
    typedef typename flann::Index<distType> FLANNIndex;
    typedef typename flann::Matrix<ScalarT> FLANNMat;


protected:
    // the "searchable" vector
    std::vector<ScalarT> v_;

    boost::shared_ptr<FLANNIndex> flann_index_;

};





#endif // FLANN1DSEARCHER_H
