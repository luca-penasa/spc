#ifndef SPC_MACROS_H
#define SPC_MACROS_H

#include <boost/foreach.hpp>
#include <limits>

// nan numbers
#define spcNANMacro std::numeric_limits<float>::quiet_NaN()

#ifndef USE_STD_SHARED_PTR
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define spcStaticPointerCast boost::static_pointer_cast
#define spcDynamicPointerCast boost::dynamic_pointer_cast

// a shared ptr
#define spcSharedPtrMacro boost::shared_ptr

// how to make one
#define spcMakeSharedPtrMacro boost::make_shared

#else // use std implementation
#include <memory>

#define spcStaticPointerCast std::static_pointer_cast
#define spcDynamicPointerCast std::dynamic_pointer_cast

// a shared ptr
#define spcSharedPtrMacro std::shared_ptr

// how to make one
#define spcMakeSharedPtrMacro std::make_shared

#endif

// a utilitu foreach from boost
#define spcForEachMacro BOOST_FOREACH

// from vtkSetMacro :-)
#define spcSetMacro(name, membername, type)                                    \
    virtual void set##name(type _arg)                                          \
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

#define WIDEN(quote) WIDEN2(quote)
#define WIDEN2(quote) #quote

#define SPC_OBJECT(classname)                                                  \
    spcTypedefSharedPtrs(classname) virtual std::string getClassName() const   \
    {                                                                          \
        return WIDEN(classname);                                               \
    }

#define spcTypedefSharedPtrs(classname)                                        \
    typedef spcSharedPtrMacro<classname> Ptr;                                  \
    typedef spcSharedPtrMacro<const classname> ConstPtr;

#endif // SPC_MACROS_H
