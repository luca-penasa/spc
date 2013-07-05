#include "images_matches_matrix.h"



vector<Match> ImageMatchesMatrix::getMatches(int ida, int idb)
{
    return matches_.at(ida*n_images_ + idb);
}


void ImageMatchesMatrix::setWorkInMirror(bool mirror)
{
    work_in_mirror_ = mirror;
}

void ImageMatchesMatrix::setNumberOfImages(size_t number)
{
    n_images_ = number;
    matches_.resize(n_images_ * n_images_);
}


void ImageMatchesMatrix::setMatches(vector<Match> matches, int ida, int idb)
{
    //simple checks
    if (ida == idb)
    {
        cout << "not really smart to set a match with itself!" << endl;
    }

    matches_.at(ida*n_images_ + idb) = matches;

    if (work_in_mirror_)
    {
        vector<Match> inverted = matches;
        for (Match &m: inverted)
            m.switchIDS();


        matches_.at(idb*n_images_ + ida) = inverted;
    }
}


bool ImageMatchesMatrix::existsMatch(int ida, int idb)
{
    vector<Match> match = getMatches(ida, idb);
    if (match.size() == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}



