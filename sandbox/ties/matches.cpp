#include "matches.h"







const bool operator_minor_on_A (const MatchXY &a, const MatchXY &b )
{
    if (a.xA_ == b.xA_)
    {
        if (a.yA_ == b.yA_)
            return (a.distance_ < b.distance_);
        else
            return (a.yA_ < b.yA_);
    }
    else
    {
        return (a.xA_ < b.xA_);
    }
}

const bool operator_equal_on_A (const MatchXY & a, const MatchXY &b)
{
    return ((a.xA_ == b.xA_) & (a.yA_ == b.yA_));
}

const bool operator_minor_on_B (const MatchXY &a, const MatchXY &b )
{
    if (a.xB_ == b.xB_)
    {
        if (a.yB_ == b.yB_)
            return (a.distance_ < b.distance_);
        else
            return (a.yB_ < b.yB_);
    }
    else
    {
        return (a.xB_ < b.xB_);
    }
}

const bool operator_equal_on_B (const MatchXY & a, const MatchXY &b)
{
    return ((a.xB_ == b.xB_) & (a.yB_ == b.yB_));
}


