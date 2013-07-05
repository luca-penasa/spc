#include "io.h"




void MatchesWriter::setMatchesAndKeypoints(vector<Match>  matches, Keypoints keysA, Keypoints keysB)
{
    keysA_ = keysA;
    keysB_ = keysB;
    matches_ = matches;

    toXYMatches();
    if (filter_duplicates_)
    {
        filterDuplicates();
    }
}

void MatchesWriter::toXYMatches()
{
    for (Match m: matches_)
    {
        MatchXY mxy;
        float xa, ya, xb, yb;
        float distance = m.distance_;

        int ida, idb;

        ida = m.idA_;
        idb = m.idB_;


        Keypoint keya = *(keysA_.keypoints_.at(ida) );
        Keypoint keyb = *(keysB_.keypoints_.at(idb) );

        mxy.xA_ = keya.x_;
        mxy.yA_ = keya.y_;
        mxy.xB_ = keyb.x_;
        mxy.yB_ = keyb.y_;
        mxy.distance_ = distance;

        matches_xy_.push_back(mxy);

    }

}

void MatchesWriter::filterDuplicates()
{
    //now we sort the pair vector with a custom rule!
    std::sort(matches_xy_.begin(), matches_xy_.end(), operator_minor_on_A);

    //now remove duplicates
    matches_xy_.erase(std::unique(matches_xy_.begin(), matches_xy_.end(), operator_equal_on_A), matches_xy_.end());

    //the same in the B-image side
    //now we sort the pair vector with a custom rule!
    std::sort(matches_xy_.begin(), matches_xy_.end(), operator_minor_on_B);

    //now remove duplicates
    matches_xy_.erase(std::unique(matches_xy_.begin(), matches_xy_.end(), operator_equal_on_B), matches_xy_.end());

}

int MatchesWriter::write()
{
    stringstream s_stream;

    ofstream out(filename_.c_str());

    for (MatchXY m : matches_xy_)
    {

        s_stream.precision(6);

        s_stream << m.xA_  << " " << m.yA_ << " " << m.xB_ << " " << m.yB_ << endl;
    }

    out << s_stream.str();
    out.close();


    return 1;
}


