#include "UniversalUniqueID.h"

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <spc/io/cereal_types.hpp>

#include <boost/lexical_cast.hpp>
namespace spc
{

UniversalUniqueID::UniversalUniqueID() : has_valid_uuid_(false)

{
}

std::string UniversalUniqueID::getUUIDAsString() const
{
    return boost::lexical_cast<std::string>(uuid_);
}

UniversalUniqueID::IDType UniversalUniqueID::operator()() const
{
    return uuid_;
}

bool UniversalUniqueID::operator==(const UniversalUniqueID &other)
{
    if (hasValidUUID() == false)
        return false;
    else
        return other.uuid_ == uuid_;
}

void UniversalUniqueID::renewUUID()
{
    uuid_ = boost::uuids::random_generator()();
    has_valid_uuid_ = true;
}

} // end nsapce
