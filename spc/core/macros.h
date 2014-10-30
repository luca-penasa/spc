#ifndef SPC_MACROS_H
#define SPC_MACROS_H


#include <limits>
#include <spc/core/macros_ptr.h>
#include <spc/core/DtiClass.h>

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
#define spcNANMacrod std::numeric_limits<double>::quiet_NaN()

// from vtkSetMacro :-)
#define spcSetMacro(name, membername, type)                                    \
    virtual void set##name(const type &_arg)                                   \
    {                                                                          \
            this->membername = _arg;                                           \
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
            this->membername = _arg;                                           \
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




#define SPC_CEREAL_REGISTER_TYPE_WITH_NAME(type, name)\
    CEREAL_REGISTER_TYPE_WITH_NAME(type, name);




#define SPC_CEREAL_REGISTER_TYPE(type)\
    CEREAL_REGISTER_TYPE(type);




#endif // SPC_MACROS_H
