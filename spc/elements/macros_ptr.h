#pragma once
#ifndef MACROS_PTR_H
#define MACROS_PTR_H

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

#define spcFwdDeclSharedPtr(name)                                              \
    class name;                                                                \
    typedef spcSharedPtrMacro<name> name##Ptr;

// how to make one
#define spcMakeSharedPtrMacro std::make_shared

#endif

#define spcTypedefSharedPtrs(classname)                                        \
    typedef spcSharedPtrMacro<classname> Ptr;                                  \
    typedef spcSharedPtrMacro<const classname> ConstPtr;


#endif // MACROS_PTR_H
