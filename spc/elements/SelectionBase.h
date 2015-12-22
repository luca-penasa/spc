#pragma once
#ifndef SELECTIONBASE_H
#define SELECTIONBASE_H
#include <spc/core/spc_eigen.h>
namespace spc {

template <class ElementT>
class SelectionBase {
public:
    typedef SelectionBase<ElementT> self_t;

    spcTypedefSharedPtrs(self_t)

        SelectionBase()
    {
    }

    virtual bool contains(const ElementT& obj) const = 0;
};

typedef SelectionBase<Eigen::Vector3f> SelectionOfPointsBase;

} //end nspace

#endif // SELECTIONBASE_H
