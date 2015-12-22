#include "ElementBase.h"
//#include <fstream>

//#include <pcl/console/print.h>
//#include <spc/elements/UniversalUniqueID.h>
#include <spc/elements/SerializableInterface.h>

#include <stack>

#include <spc/core/logging.h>



namespace spc {

DtiClassType ElementBase::Type("ElementBase", nullptr); // it is a root

ElementBase::Ptr ElementBase::getPtr()
{
    try {
        return shared_from_this();
    }
    catch (std::exception& e) {
        LOG(ERROR) << "shared_from_this raised exception, meybe you are calling getPtr() from the object constructor? exception was " << e.what();
        return nullptr;
    }
}

void ElementBase::addChild(ElementBase::Ptr child)
{
    if (!child) {
        LOG(ERROR) << "Passed pointer is null";
        return;
    }

    LOG(INFO) << "going to add element " << child;

    spc::ElementBase::Ptr myptr = getPtr();

    LOG(INFO) << "myptr " << myptr;
    child->setParent(myptr);
    childs_.push_back(child);

    LOG(INFO) << "added element " << child;
}

std::vector<ElementBase::Ptr> ElementBase::findElementsThatAre(const DtiClassType* dti)
{
    std::vector<ElementBase::Ptr> out;

    std::stack<ElementBase::Ptr> tocheck;
    tocheck.push(this->getPtr());

    while (!tocheck.empty()) {
        ElementBase::Ptr thisel = tocheck.top();
        tocheck.pop();

        if (thisel->isA(dti)) {
            out.push_back(thisel);
        }

        for (spc::ElementBase::Ptr child : thisel->getChilds()) {
            tocheck.push(child);
        }
    }

    return out;
}

std::vector<ElementBase::Ptr> ElementBase::getChildsThatAre(const DtiClassType* dti) const
{

    std::vector<ElementBase::Ptr> out;
    for (ElementBase::Ptr obj : getChilds()) {
        if (obj->isA(dti))
            out.push_back(obj);
    }

    return out;
}

void ElementBase::removeChild(ElementBase::Ptr child)
{
    childs_.erase(std::remove(childs_.begin(), childs_.end(), child), childs_.end());

    // we are not parent of this child anymore
    child->setParent(NULL);
}

void ElementBase::setParent(const ElementBase::Ptr parent)
{
    if (!parent) {
        LOG(ERROR) << "Passed pointer is null";
        return;
    }
    parent_ = parent;

    LOG(INFO) << "set parent " << parent;
}

std::vector<ElementBase::Ptr> ElementBase::findElementsInParents(const DtiClassType* dti) const
{
    std::vector<ElementBase::Ptr> out;
    spc::ElementBase::Ptr current_parent = this->getParent();

    while (current_parent != NULL) {
        if (current_parent->isA(dti)) {
            out.push_back(current_parent);
        }

        current_parent = current_parent->getParent();
    }

    return out;
}

} // end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::ElementBase)
