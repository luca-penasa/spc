#pragma once
#ifndef UNIVERSALUNIQUEID_H
#define UNIVERSALUNIQUEID_H

#include <spc/elements/spcObject.h>
#include <boost/uuid/uuid.hpp>

namespace  spc {

class UniversalUniqueID: public spcObject
{
public:
    SPC_OBJECT(UniversalUniqueID)

    typedef typename  boost::uuids::uuid IDType;

    UniversalUniqueID();

    spcGetMacro(HasValidUUID, has_valid_uuid_, bool)

    spcGetMacro(UUID, uuid_, IDType)

    std::string getUUIDAsString() const;

    IDType operator () () const;

    bool operator == (const UniversalUniqueID &other);

protected:
    void renewUUID();


    spcSetMacro(UUID, uuid_, IDType)

protected:
    boost::uuids::uuid uuid_;

    bool has_valid_uuid_;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(cereal::base_class<spcObject>( this ),
            CEREAL_NVP(uuid_),
            CEREAL_NVP(has_valid_uuid_) );
    }

};

}//end nspace

#endif // UNIVERSALUNIQUEID_H
