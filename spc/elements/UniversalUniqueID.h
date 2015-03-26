#pragma once
#ifndef UNIVERSALUNIQUEID_H
#define UNIVERSALUNIQUEID_H

#include <boost/uuid/uuid.hpp>
#include <spc/core/macros.h>
#include <cereal/cereal.hpp>
namespace spc
{

	class SPC_LIB_API UniversalUniqueID
{
public:
    spcTypedefSharedPtrs(UniversalUniqueID)

    typedef boost::uuids::uuid IDType;

    UniversalUniqueID();

    bool hasValidUUID() const
    {
        return has_valid_uuid_;
    }

    std::string getUUIDAsString() const;

    IDType operator()() const;

    bool operator==(const UniversalUniqueID &other);

protected:
    void renewUUID();

    void setUUID(IDType uuid)
    {
        uuid_ = uuid;
    }

protected:
    boost::uuids::uuid uuid_;

    bool has_valid_uuid_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(uuid_),
           CEREAL_NVP(has_valid_uuid_));
    }
};

} // end nspace

#endif // UNIVERSALUNIQUEID_H
