#ifndef GEOLOGIC_ELEMENT_BASE_H
#define GEOLOGIC_ELEMENT_BASE_H

#include <boost/shared_ptr.hpp>


namespace spc
{

class GeologicElementBase
{

public:
    typedef typename boost::shared_ptr<GeologicElementBase> Ptr;

    GeologicElementBase();
};


} //end nspace

#endif // GEOLOGIC_OBJECT_H
