#ifndef GENERICFILTER_H
#define GENERICFILTER_H

#include <spc/core/common.h>
#include <spc/core/ElementBase.h>
#include <vector>
namespace spc
{

struct InputRequirements
{


    std::vector<spc::DtiClassType *> types;
    size_t n_elements = 1;
};

class GenericFilter
{
public:

    spcTypedefSharedPtrs(GenericFilter)

    GenericFilter();

    virtual void doComputations() = 0;

    spcSetGetMacro(Name, name_, std::string )


    virtual InputRequirements getInputRequirements() const = 0;

    virtual void setInput(std::vector<spc::ElementBase::Ptr> inelements);

    virtual void setInput(spc::ElementBase::Ptr inobj);

    virtual bool canCompute() const;

    virtual void clearOutput();


    virtual std::vector<spc::ElementBase::Ptr>  getOutput() const ;
protected:


    std::string name_;

     std::vector<spc::ElementBase::Ptr> inelements_;

     std::vector<spc::ElementBase::Ptr> outelements_;

};

}

#endif // GENERICFILTER_H
