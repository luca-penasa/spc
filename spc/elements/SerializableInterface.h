#ifndef SERIALIZABLEINTERFACE_H
#define SERIALIZABLEINTERFACE_H

//#include <cereal/cereal.hpp>
#include <spc/core/macros_ptr.h>
#include <spc/core/logging.h>

namespace spc
{

class EigenTable;
typedef spcSharedPtrMacro<EigenTable> EigenTablePtr;

class ISerializable
{
public:
    spcTypedefSharedPtrs(ISerializable)



    ISerializable();

    ~ISerializable()
    {

    }


    virtual bool isSerializable() const = 0;

    virtual bool isAsciiSerializable() const = 0;

    virtual int toAsciiStream(std::ostream &stream) const
    {
        LOG(WARNING) << "ascii serialization in base class has been called but nothing done. "
                        "this method must be overridden as needed";

        return -1;
    }

    virtual EigenTablePtr asEigenTable() const;
};

} // end nspace

#endif // SERIALIZABLEINTERFACE_H
