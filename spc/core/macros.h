#pragma once
#ifndef SPC_MACROS_H
#define SPC_MACROS_H

#include <limits>
#include <spc/core/macros_ptr.h>
#include <spc/core/DtiClass.h>

// experimental DTICLASS macro runtyme type exposer
#define EXPOSE_TYPE                                           \
    static DtiClassType Type;                                 \
    virtual DtiClassType* getType() const override            \
    {                                                         \
        return &Type;                                         \
    }                                                         \
    virtual bool isA(const DtiClassType* type) const override \
    {                                                         \
        return this->getType()->isA(type);                    \
    }

#define EXPOSE_TYPE_BASE                             \
    static DtiClassType Type;                        \
    virtual DtiClassType* getType() const            \
    {                                                \
        return &Type;                                \
    }                                                \
    virtual bool isA(const DtiClassType* type) const \
    {                                                \
        return this->getType()->isA(type);           \
    }

// nan numbers
#define spcNANMacro std::numeric_limits<float>::quiet_NaN()
#define spcNANMacrod std::numeric_limits<double>::quiet_NaN()
#define spcNANMacros std::numeric_limits<size_t>::quiet_NaN()

// from vtkSetMacro :-)
#define spcSetMacro(name, membername, type)  \
    virtual void set##name(const type& _arg) \
    {                                        \
        this->membername = _arg;             \
    }

// from vtkGetMacro :-)
#define spcGetMacro(name, membername, type) \
    virtual type get##name() const          \
    {                                       \
        return this->membername;            \
    }

#define spcSetGetMacro(name, membername, type) \
    spcSetMacro(name, membername, type)        \
        spcGetMacro(name, membername, type)

#define spcSetObjectMacro(name, membername, type) \
    virtual void set##name(type::Ptr _arg)        \
    {                                             \
        this->membername = _arg;                  \
    }                                             \
    virtual bool has##name() const                \
    {                                             \
        return (bool)this->membername;            \
    }

#define spcGetObjectMacro(name, membername, type) \
    virtual type::Ptr get##name() const           \
    {                                             \
        return this->membername;                  \
    }

#define spcAddClone(classname)                         \
    virtual ElementBase::Ptr clone() const override    \
    {                                                  \
        return ElementBase::Ptr(new classname(*this)); \
    }

#define SPC_ELEMENT(classname) spcTypedefSharedPtrs(classname) spcAddClone(classname)

#define SPC_CEREAL_REGISTER_TYPE_WITH_NAME(type, name) \
    CEREAL_REGISTER_TYPE_WITH_NAME(type, name);

#define SPC_CEREAL_REGISTER_TYPE(type) \
    CEREAL_REGISTER_TYPE(type);

//! just until some stuff in the dev branch of cereal is not merged with master
#define SPC_CEREAL_CLASS_VERSION(TYPE, VERSION_NUMBER)                                                                                              \
    namespace cereal {                                                                                                                              \
    namespace detail {                                                                                                                              \
        template <> struct Version<TYPE> {                                                                                                          \
            static const std::uint32_t version;                                                                                                     \
            static std::uint32_t registerVersion()                                                                                                  \
            {                                                                                                                                       \
                ::cereal::detail::StaticObject<Versions>::getInstance().mapping.emplace(std::type_index(typeid(TYPE)).hash_code(), VERSION_NUMBER); \
                return VERSION_NUMBER;                                                                                                              \
            }                                                                                                                                       \
        }; /* end Version */                                                                                                                        \
        const std::uint32_t Version<TYPE>::version = Version<TYPE>::registerVersion();                                                              \
    }                                                                                                                                               \
    } // end namespaces

//#ifdef _WIN32
//	#ifdef SPC_LIB_EXPORTS
//		#define SPC_LIB_API __declspec(dllexport)
//	#else
//		#define SPC_LIB_API __declspec(dllimport)
//	#endif
//#else
//	#define SPC_LIB_API
//#endif

#endif // SPC_MACROS_H
