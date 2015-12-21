#pragma once
#ifndef MACROS_PTR_H
#define MACROS_PTR_H


#include <memory>

#define spcStaticPointerCast std::static_pointer_cast
#define spcDynamicPointerCast std::dynamic_pointer_cast

// a shared ptr
#define spcSharedPtrMacro std::shared_ptr

#define spcFwdDeclSharedPtr(name)                                              \
    class name;                                                                \
    typedef spcSharedPtrMacro<name> name##Ptr;                                  \
    typedef spcSharedPtrMacro<const name> name##ConstPtr;

// how to make one
#define spcMakeSharedPtrMacro std::make_shared


#define spcTypedefSharedPtrs(classname)                                        \
    typedef spcSharedPtrMacro<classname> Ptr;                                  \
    typedef spcSharedPtrMacro<const classname> ConstPtr;





#endif // MACROS_PTR_H
