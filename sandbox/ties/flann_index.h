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
public:
    FlannIndex() {}

    void buildIndex()
    {
        cout << "building index..." << endl;
        index_->buildIndex();
        cout << "built!" << endl;
    }

    void setInputKeypoints(Keypoints::Ptr kpoints)
    {
        keypoints_ = kpoints;

        desc_as_v_ = keypoints_->getAllDescriptorAsStdVector<ScalarT>();
        flann_descriptors_ = flann::Matrix<ScalarT>(&desc_as_v_[0], keypoints_->getNumberOfKeypoints(), keypoints_->getSizeOfFeature());

        flann::KDTreeIndexParams params(4);
        //        flann::AutotunedIndexParams params;


        index_ = flannIndexTypePtr( new flannIndexType(flann_descriptors_, params) );
        //        index_ = flannIndexTypePtr( new flannIndexType(flann_descriptors_, flann::AutotunedIndexParams()) );
    }

    vector<vector<Match> > getMatchesMulti(Keypoints::Ptr points, int nn)
    {
        last_query_ = points->getAllDescriptorAsStdVector<ScalarT>();
        flann::Matrix<ScalarT> query_flann (&last_query_[0], points->getNumberOfKeypoints(), points->getSizeOfFeature());

        vector<int> ids(points->getNumberOfKeypoints() * nn);
        vector<float> dists(points->getNumberOfKeypoints() * nn);

        flann::Matrix<int> flann_ids (ids.data(), points->getNumberOfKeypoints(), nn);
        flann::Matrix<float> flann_dists(dists.data(), points->getNumberOfKeypoints(), nn);


        cout << "data points: " << this->keypoints_->keypoints_.size() ;
        cout << " query points: " << points->getNumberOfKeypoints() << endl;


        clock_t begin, end;
        begin = clock();

        //        spars.cores = 3;
        index_->knnSearch(query_flann, flann_ids, flann_dists, nn, flann::SearchParams());

        end = clock();
        double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
        cout << time_spent << endl;


        vector<vector<Match> > matches;


        //putting in matches
        for (int i = 0; i < points->getNumberOfKeypoints() ; ++i)
        {
            vector<Match> this_matches;
            for (int j=0; j < nn; ++j)
            {
                Match match(ids.at(i*nn+j), i);
                match.distance_ = dists.at(i*nn + j);
                this_matches.push_back(match);
            }

            matches.push_back(this_matches);


        }

        cout << "Got " << matches.size() << " matches!" << endl;
        return matches;
    }


    void saveIndexToFile(string filename)
    {
        index_->save(filename);
    }

    void loadIndexFromFile(string filename)
    {
        index_ = flannIndexTypePtr ( new flannIndexType (flann_descriptors_, flann::SavedIndexParams(filename)) );
    }


    Keypoints::Ptr getTrainKeys()
    {
        return keypoints_;
    }

    flannIndexTypePtr index_;
    Keypoints::Ptr keypoints_;

    flann::Matrix<ScalarT> flann_descriptors_;

    vector<ScalarT> desc_as_v_;
    vector<ScalarT> last_query_;

    vector<vector<Keypoint> > matches_;

};


#endif // FLANN_INDEX_H
