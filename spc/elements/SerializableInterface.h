#ifndef SERIALIZABLEINTERFACE_H
#define SERIALIZABLEINTERFACE_H

//#include <cereal/cereal.hpp>
#include <spc/elements/macros_ptr.h>

namespace spc
{

class EigenTable;
typedef spcSharedPtrMacro<EigenTable> EigenTablePtr;

class ISerializable
{
public:
    spcTypedefSharedPtrs(ISerializable)

    ISerializable();

    virtual bool isSerializable() const = 0;

    virtual bool isAsciiSerializable() const = 0;

    virtual EigenTablePtr asEigenTable() const;
};

} // end nspace

#endif // SERIALIZABLEINTERFACE_H
