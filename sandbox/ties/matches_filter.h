#ifndef MATCHES_FILTER_H
#define MATCHES_FILTER_H

//STD
#include <vector>
#include <limits>

//LOCAL
#include "matches.h"


using namespace std;

///
/// \brief The MatchesFilter class
///
class MatchesFilter
{

public:
    enum FILTER_TYPE {MINIMUM, FIRST_NEAREST};

    MatchesFilter() {filter_type_ = MINIMUM; }

    void setInputMatches(vector<vector<Match> > multi_matches)
    {
        in_matches_ = multi_matches;
    }

    void setFilterType(FILTER_TYPE type)
    {
        filter_type_ = type;
    }

    void filter(vector<Match> &matches, float factor)
    {
        switch (filter_type_)
        {
            case MINIMUM:
            {
            float old_dist = std::numeric_limits<float>::max();
            for (int i = 0 ; i < in_matches_.size(); ++i)
            {
                Match match = in_matches_.at(i).at(0);
                float dist = match.distance_;
                if (old_dist > dist )
                    old_dist = dist;

            }

            float min_distance = old_dist;
            cout << "Found min distance: " << min_distance <<  endl;

            float discriminant_distance = factor * min_distance;
            for (int i = 0 ; i < in_matches_.size(); ++i)
            {
                Match match = in_matches_.at(i).at(0);
                if (match.distance_ < discriminant_distance)
                {
                    matches.push_back(match);
                }

            }



            break;
            }


            case FIRST_NEAREST:
            {

            //filtering!
            for (int i = 0; i < in_matches_.size(); ++i)
            {
                Match m1 = in_matches_.at(i).at(0);
                Match m2 = in_matches_.at(i).at(1);

//                int nn = in_matches_.at(0).size();
                if (m1.distance_ < m2.distance_ * factor)
                {

                    matches.push_back(m1);
                }

            }

                break;
            }
            default:
            {
                break;
            }
        }
    }


    vector<vector<Match> > in_matches_;
    FILTER_TYPE filter_type_;

};



#endif // MATCHES_FILTER_H
