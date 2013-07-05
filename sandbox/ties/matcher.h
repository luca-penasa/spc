#ifndef MATCHER_H
#define MATCHER_H

#include <string>
#include <vector>


#include "matches.h"
#include "keypoints.h"
#include "flann_index.h"
#include "image_matches_mask.h"
#include "images_matches_matrix.h"

using namespace std;


template <typename ScalarT>
class Matcher
{
public:

    typedef  FlannIndex<ScalarT> FlannIndexType;
    typedef typename FlannIndex<ScalarT>::Ptr FlannIndexTypePtr;

    Matcher() ;

    void setFilenames(vector<string> fnames);

    void updateFinders();

    void updateMatches();

    void updateKeypoints();

    void setCacheDir(string dir);

    void setMatchingMask(ImageMatchesMask &mask);

    void writeOutHomolFiles();

    void setOutputDirectory(string out_dir) {out_dir_ = out_dir;}

private:

    string out_dir_;

    vector<string> filenames_;

    vector< typename FlannIndexType::Ptr > finders_;

    vector<Keypoints::Ptr> keypoints_;

    ImageMatchesMatrix image_matrix_;

    string cache_dir_;

    ImageMatchesMask mask_;
};



#endif // MATCHER_H
