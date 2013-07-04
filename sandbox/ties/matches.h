#ifndef MM_MATCHES_H
#define MM_MATCHES_H

#include <memory>
#include <vector>


using namespace std;


class Match
{

public:

    typedef shared_ptr <Match> Ptr;
    Match() {}

    Match(int idA, int idB) {idA_ = idA; idB_ = idB;}

    void switchIDS()
    {
        int tmp = idA_;
        idA_ = idB_;
        idB_ = tmp;

    }

    int idA_, idB_;
    float distance_;

};

//class Matches
//{

//public:
//    typedef shared_ptr <Matches> Ptr;
//    Matches() {}

//    vector<Match::Ptr> matches_;
//};

///
/// \brief The ImageMatchesMask class
///
class ImageMatchesMask
{
public:
    ImageMatchesMask() {}

    void setNumberOfImages(size_t n)
    {
        n_images_ = n;
        mask_.assign(n_images_ * n_images_, false);
    }

    void setElement(int i, int j, bool val)
    {
        mask_.at(i*n_images_ + j) = val;
    }

    bool getElement(int i, int j)
    {
        return mask_.at(i*n_images_ + j);
    }

    void setUpperTriangularNoDiagonal()
    {
        for (int i = 0; i < n_images_ ; ++i)
            for (int j = 0; j < i; ++j)
                mask_.at(i*n_images_ + j) = true;


    }



    vector<bool> mask_;

    size_t n_images_;

};

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

    vector<Match> getMatches(int ida, int idb)
    {
        return matches_.at(ida*n_images_ + idb);
    }

    void setMatches(vector<Match> matches, int ida, int idb)
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

    void setWorkInMirror(bool mirror)
    {
        work_in_mirror_ = mirror;
    }

    void setNumberOfImages(size_t number)
    {
        n_images_ = number;
        matches_.resize(n_images_ * n_images_);
    }

    bool existsMatch(int ida, int idb)
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


    size_t n_images_;
    bool work_in_mirror_;

    vector< vector<Match> > matches_;

    vector<string> filenames_;
};





#endif // MM_MATCHES_H
