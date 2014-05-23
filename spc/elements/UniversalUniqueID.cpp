#include "UniversalUniqueID.h"

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <spc/common/cereal_types.hpp>

#include <boost/lexical_cast.hpp>
namespace spc
{


UniversalUniqueID::UniversalUniqueID() : has_valid_uuid_(false)

{
    this->renewUUID();
}


std::string UniversalUniqueID::getUUIDAsString() const
{
    return boost::lexical_cast<std::string>(getUUID());
}

UniversalUniqueID::IDType UniversalUniqueID::operator ()() const
{
    return getUUID();
}

bool UniversalUniqueID::operator ==(const UniversalUniqueID &other)
{
    if (getHasValidUUID() ==  false)
        return false;
    else
        return other.getUUID() == this->getUUID();
}

void UniversalUniqueID::renewUUID()
{
    uuid_ = boost::uuids::random_generator()();
    has_valid_uuid_ = true;
}



}//end nsapce


