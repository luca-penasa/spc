#include "UniversalUniqueObject.h"

#include <boost/uuid/uuid_generators.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>



namespace  spc
{

UniversalUniqueObject::UniversalUniqueObject()
    : tag(boost::uuids::random_generator()())
{}

UniversalUniqueObject::UniversalUniqueObject(int state)
    : tag(boost::uuids::random_generator()())

{}

UniversalUniqueObject::UniversalUniqueObject(const UniversalUniqueObject &rhs)
    : tag(rhs.tag)

{}

bool UniversalUniqueObject::operator==(const UniversalUniqueObject &rhs) const {
    return tag == rhs.tag;
}

UniversalUniqueObject &UniversalUniqueObject::operator=(const UniversalUniqueObject &rhs) {
    tag = rhs.tag;
}

boost::uuids::uuid UniversalUniqueObject::getUUID() const
{
    return tag;
}

std::string UniversalUniqueObject::getUUIDAsString() const
{
    return boost::lexical_cast<std::string>(getUUID());
}




} // end nspace
