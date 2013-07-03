#ifndef MM_MATCHES_H
#define MM_MATCHES_H

#include <memory>
using namespace std;


class Match
{

public:

    typedef shared_ptr <Match> Ptr;
    Match() {}

    Match(int idA, int idB) {idA_ = idA; idB_ = idB;}

    int idA_, idB_;
    float distance_;
};

class Matches
{

public:
    typedef shared_ptr <Matches> Ptr;
    Matches() {}

    vector<Match::Ptr> matches_;
};



#endif // MM_MATCHES_H
