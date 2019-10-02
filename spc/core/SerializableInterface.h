#pragma once
#ifndef SERIALIZABLEINTERFACE_H
#define SERIALIZABLEINTERFACE_H

#include <spc/core/macros_ptr.h>
#include <spc/core/spc_cereal.hpp>

namespace spc {

class EigenTable;
typedef spcSharedPtrMacro<EigenTable> EigenTablePtr;

class ISerializable {
public:
    spcTypedefSharedPtrs(ISerializable)

        ISerializable();

    ~ISerializable()
    {
    }

    virtual bool isSerializable() const = 0;

    virtual bool isAsciiSerializable() const = 0;

    virtual int toAsciiStream(std::ostream& stream) const;

    virtual EigenTablePtr asEigenTable() const;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
//        ar(cereal::base_class<spc::ISerializable>(this),
//                    CEREAL_NVP(name_),
//            CEREAL_NVP(childs_));
    }
};

} // end nspace

#endif // SERIALIZABLEINTERFACE_H
