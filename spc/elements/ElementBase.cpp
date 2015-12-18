#include "ElementBase.h"
//#include <fstream>

//#include <pcl/console/print.h>
#include <spc/elements/UniversalUniqueID.h>
#include <spc/elements/SerializableInterface.h>
namespace spc
{

DtiClassType ElementBase::Type ("ElementBase", nullptr); // it is a root


ElementBase::Ptr ElementBase::getPtr()
{
    try
    {
        return shared_from_this();
    }
    catch (std::exception& e)
    {
       LOG(ERROR) << "shared_from_this raised exception, meybe you are calling getPtr() from the object constructor? exception was " << e.what() ;
    }
}

void ElementBase::update()
{
    // do stuff in subclasses
    LOG(INFO) << "Called update in base method, update() not implemented";
    modified_ = false;
}

UniversalUniqueID ElementBase::getUniversalUUID() const
{
    return universal_id_;
}

void ElementBase::addChild(ElementBase::Ptr child)
{
    if (!child)
    {
        LOG(ERROR) << "Passed pointer is null";
        return;
    }

    LOG(INFO) << "going to add element " << child;


   spc::ElementBase::Ptr myptr = getPtr();

    LOG(INFO) <<  "myptr " << myptr;
    child->setParent(myptr);
    childs_.push_back(child);

    LOG(INFO) << "added element " << child;
}

void ElementBase::removeChild(ElementBase::Ptr child)
{
    childs_.erase(std::remove(childs_.begin(), childs_.end(), child), childs_.end());

    // we are not parent of this child anymore
    child->setParent(NULL);
}

void ElementBase::setParent(const ElementBase::Ptr parent)
{
    if(!parent)
    {
        LOG(ERROR) << "Passed pointer is null";
        return;
    }
    parent_ = parent;

    LOG(INFO) << "set parent " << parent;
}

std::vector<ElementBase::Ptr> ElementBase::findElementsInParents(const DtiClassType *dti) const
{
    std::vector<ElementBase::Ptr> out;
    spc::ElementBase::Ptr current_parent = this->getParent();

    while (current_parent != NULL)
    {
        if (current_parent->isA(dti))
        {
            out.push_back(current_parent);
        }

        current_parent = current_parent->getParent();
    }

    return out;
}



} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::ElementBase)
//SPC_CEREAL_CLASS_VERSION( spc::ElementBase, 1 )
