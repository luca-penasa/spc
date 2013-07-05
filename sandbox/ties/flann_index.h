#ifndef FLANN_INDEX_H
#define FLANN_INDEX_H

#include <flann/flann.hpp>
#include <memory>
#include "keypoints.h"

using namespace std;

template <typename ScalarT>
class FlannIndex
{
public:

    typedef typename flann::L2<ScalarT> flannDistanceType;
    typedef flann::Index<flannDistanceType > flannIndexType;
    typedef shared_ptr<flannIndexType> flannIndexTypePtr;

    typedef std::shared_ptr<FlannIndex> Ptr;

    FlannIndex() {}

    void buildIndex();


    void setInputKeypoints(Keypoints::Ptr kpoints);


    vector<vector<Match> > getMatchesMulti(Keypoints::Ptr points, int nn);



    void saveIndexToFile(string filename);

    void loadIndexFromFile(string filename);



    Keypoints::Ptr getTrainKeys();


private:
    flannIndexTypePtr index_;
    Keypoints::Ptr keypoints_;

    flann::Matrix<ScalarT> flann_descriptors_;

    vector<ScalarT> desc_as_v_;
    vector<ScalarT> last_query_;

    vector<vector<Keypoint> > matches_;

};


#endif // FLANN_INDEX_H
