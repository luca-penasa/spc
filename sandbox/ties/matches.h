#ifndef MM_MATCHES_H
#define MM_MATCHES_H

#include <memory>
#include <vector>
#include <iostream>


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


///
/// \brief The MatchXY class
/// NOT USE THIS CLASS FOR OTHER PORPUSES THAN SORTING AND CLEANING FROM DUPLICATES!
/// it features an uncommon operator overloading that may lead to misunderstandings!
///
class MatchXY
{
public:
    MatchXY () {}

    MatchXY(float xa, float ya, float xb,float yb, float distance)
    {
        xA_ = xa;
        yA_ = ya;
        xB_ = xb;
        yB_ = yb;
        distance_ = distance; //feature space distance for this match
    }

    float xA_, yA_, xB_, yB_, distance_;

};

const bool operator_minor_on_A (const MatchXY &a, const MatchXY &b );

const bool operator_equal_on_A (const MatchXY & a, const MatchXY &b);

const bool operator_minor_on_B (const MatchXY &a, const MatchXY &b );

const bool operator_equal_on_B (const MatchXY & a, const MatchXY &b);






#endif // MM_MATCHES_H
