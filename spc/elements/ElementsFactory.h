#pragma once
#ifndef ELEMENTSFACTORY_H
#define ELEMENTSFACTORY_H

#include <spc/core/macros.h>
namespace spc
{
class ElementBase;
typedef spcSharedPtrMacro<ElementBase> ElementBasePtr;

class SPC_LIB_API  ElementsFactory
{
public:
    ElementBasePtr create(const std::string name);
};

}

#endif // ELEMENTSFACTORY_H
