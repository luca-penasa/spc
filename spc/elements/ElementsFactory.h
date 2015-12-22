#pragma once
#ifndef ELEMENTSFACTORY_H
#define ELEMENTSFACTORY_H

#include <spc/core/macros.h>
namespace spc {

spcFwdDeclSharedPtr(ElementBase)

    class ElementsFactory {
public:
    ElementBasePtr create(const std::string name);
};
}

#endif // ELEMENTSFACTORY_H
