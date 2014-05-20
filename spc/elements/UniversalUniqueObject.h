#pragma once
#ifndef UNIVERSALUNIQUEOBJECT_H
#define UNIVERSALUNIQUEOBJECT_H

#include <string>
#include <boost/uuid/uuid.hpp>

#include <spc/common/macros.h>



namespace spc
{

class UniversalUniqueObject
{
public:
    spcTypedefSmartPointersMacro(UniversalUniqueObject)

    UniversalUniqueObject();

    explicit UniversalUniqueObject(int state);

    UniversalUniqueObject(UniversalUniqueObject const& rhs);

    bool operator==(UniversalUniqueObject const& rhs) const;

    UniversalUniqueObject& operator=(UniversalUniqueObject const& rhs);

    boost::uuids::uuid getUUID() const;

    std::string getUUIDAsString() const;

private:
    boost::uuids::uuid tag;



};


} // end nspace
#endif // UNIVERSALUNIQUEOBJECT_H
