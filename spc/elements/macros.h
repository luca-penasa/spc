#ifndef SPC_MACROS_H
#define SPC_MACROS_H

#include <boost/foreach.hpp>
#include <limits>
#include <spc/elements/macros_ptr.h>
#include <spc/elements/DtiClass.h>

// experimental DTICLASS macro runtyme type exposer
#define EXPOSE_TYPE                                                            \
    static DtiClassType Type;                                                  \
    virtual DtiClassType *getType() const                                      \
    {                                                                          \
        return &Type;                                                          \
    }                                                                          \
    virtual bool isA(const DtiClassType *type) const                           \
    {                                                                          \
        return this->getType()->isA(type);                                     \
    }

// nan numbers
#define spcNANMacro std::numeric_limits<float>::quiet_NaN()

// a utilitu foreach from boost
#define spcForEachMacro BOOST_FOREACH

// from vtkSetMacro :-)
#define spcSetMacro(name, membername, type)                                    \
    virtual void set##name(const type &_arg)                                   \
    {                                                                          \
        if (this->membername != _arg) {                                        \
            this->membername = _arg;                                           \
            this->modified();                                                  \
        }                                                                      \
    }

// from vtkGetMacro :-)
#define spcGetMacro(name, membername, type)                                    \
    virtual type get##name() const                                             \
    {                                                                          \
        return this->membername;                                               \
    }

#define spcSetObjectMacro(name, membername, type)                              \
    virtual void set##name(type::Ptr _arg)                                     \
    {                                                                          \
        if (this->membername != _arg) {                                        \
            this->membername = _arg;                                           \
            this->modified();                                                  \
        }                                                                      \
    }                                                                          \
    virtual bool has##name() const                                             \
    {                                                                          \
        return (bool)this->membername;                                         \
    }

#define spcGetObjectMacro(name, membername, type)                              \
    virtual type::Ptr get##name() const                                        \
    {                                                                          \
        return this->membername;                                               \
    }

#define SPC_OBJECT(classname) spcTypedefSharedPtrs(classname)

#endif // SPC_MACROS_H
