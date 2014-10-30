#ifndef CEREAL_TYPES_HPP
#define CEREAL_TYPES_HPP
#include <boost/uuid/uuid.hpp>

namespace cereal
{
template <class Archive> inline
void
save (Archive & ar, const boost::uuids::uuid &v)
{

    for (size_t i = 0; i < v.static_size(); ++i)
        ar( v.data[i] );

}

template <class Archive> inline
void
load (Archive & ar, boost::uuids::uuid &v)
{
    for (size_t i = 0; i < v.static_size(); ++i)
        ar( v.data[i] );
}



}//end nspace

#endif // CEREAL_TYPES_HPP
