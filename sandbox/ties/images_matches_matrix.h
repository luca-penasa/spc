#ifndef IMAGES_MATCHES_MATRIX_H
#define IMAGES_MATCHES_MATRIX_H

#include <vector>

#include "matches.h"

using namespace std;

///
/// \brief The ImageMatchesMatrix class
///
class ImageMatchesMatrix
{
public:
    ImageMatchesMatrix(size_t n_images) {
        work_in_mirror_ = true;
        setNumberOfImages(n_images);
    }

    vector<Match> getMatches(int ida, int idb);


    void setMatches(vector<Match> matches, int ida, int idb);


    void setWorkInMirror(bool mirror);


    void setNumberOfImages(size_t number);


    bool existsMatch(int ida, int idb);

private:
    size_t n_images_;

    bool work_in_mirror_;

    vector< vector<Match> > matches_;

    vector<string> filenames_;
};



#endif // IMAGES_MATCHES_MATRIX_H
