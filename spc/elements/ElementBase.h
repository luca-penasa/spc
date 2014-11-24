#pragma once
#ifndef SPC_OBJECT_H
#define SPC_OBJECT_H

#include <spc/core/macros.h>
//#include <cereal/cereal.hpp>
#include <spc/elements/UniversalUniqueID.h>
#include <spc/elements/SerializableInterface.h>
#include <spc/elements/ElementWithVariantProperties.h>
#include <stack>
#include <spc/core/logging.h>

namespace spc
{


class ElementBase : public ISerializable, public ElementWithVariantProperties, public std::enable_shared_from_this<ElementBase>
{
public:
    spcTypedefSharedPtrs(ElementBase)
    EXPOSE_TYPE
    ElementBase()
    {
    }

    Ptr getPtr()
    {
           return shared_from_this();
    }

    ElementBase (const ElementBase& other): ElementWithVariantProperties(other)
    {
        // for now we copy alse the uuid
        this->universal_id_ = other.getUniversalUUID();
        this->modified_ = other.modified_;
    }

    virtual ~ElementBase()
    {

    }

    //! a generic call to a virtual update() method
    //! some elements may depend upon other elements
    //! if this is reimplemented we have a way to actually update internals stuff

    virtual void update();

    virtual void modified()
    {
        this->modified_ = true;
    }

    /// this should also be present in the interface for elements with variant
    /// properties
    virtual bool hasVariantProperties() const
    {
        return true;
    }

    virtual ElementBase::Ptr clone() const
    {
        DLOG(WARNING) << "called clone in ElementBase class. This method must be re-implemented in derived classes";
        return ElementBase::Ptr(new ElementBase(*this));
    }






    virtual UniversalUniqueID getUniversalUUID() const;

    std::string getElementName() const
    {
        return name_;
    }

    void setElementName(const std::string &name)
    {
        name_ = name;
    }

    //!\todo this could go in a separated interface
    void addChild(ElementBase::Ptr child)
    {
        child->setParent(this->getPtr());
        childs_.push_back(child);
    }

    std::vector<ElementBase::Ptr> getChilds() const
    {
        return childs_;
    }

    //! only at the first level
    std::vector<ElementBase::Ptr> getChildsThatAre(const DtiClassType * dti) const
    {

        std::vector<ElementBase::Ptr> out;
        for (ElementBase::Ptr obj: getChilds())
        {
            if (obj->isA(dti))
                out.push_back(obj);
        }

        return out;
    }

    template<class Type>
    std::vector<typename Type::Ptr> getChildsThatAre(const DtiClassType * dti) const
    {
        std::vector<typename Type::Ptr> out;
        std::vector<spc::ElementBase::Ptr> el = this->getChildsThatAre(dti);
        for (spc::ElementBase::Ptr e: el)
        {
            typename Type::Ptr astype = spcDynamicPointerCast<Type> (e);

            if (astype)
                out.push_back(astype);
        }

        return out;

    }


    std::vector<ElementBase::Ptr> findElementsThatAre(const DtiClassType * dti)
    {
        std::vector<ElementBase::Ptr> out;

        std::stack<ElementBase::Ptr> tocheck;
        tocheck.push(this->getPtr());

        while (!tocheck.empty())
        {
            ElementBase::Ptr thisel = tocheck.top();
            tocheck.pop();

            if (thisel->isA(dti))
            {
                out.push_back(thisel);
            }

            for (spc::ElementBase::Ptr child: thisel->getChilds())
            {
                tocheck.push(child);
            }

        }

        return out;
    }


    std::vector<ElementBase::Ptr> findElementsInParents(const DtiClassType * dti) const
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


    template <class T>
    std::vector<typename T::Ptr> findElementsInParents(const DtiClassType * dti) const
    {
        std::vector<ElementBase::Ptr> good = findElementsInParents(dti);

        std::vector<typename T::Ptr> out;
        for (ElementBase::Ptr el: good)
        {
            typename T::Ptr outel = spcDynamicPointerCast<T> (el);

            if (outel)
                out.push_back(outel);
        }

        return out;
    }


    template<class T>
    std::vector<typename T::Ptr> findElementsThatAre(const DtiClassType * dti)
    {
        std::vector<ElementBase::Ptr> good = findElementsThatAre(dti);

        std::vector<typename T::Ptr> out;
        for (ElementBase::Ptr el: good)
        {
            typename T::Ptr outel = spcDynamicPointerCast<T> (el);

            if (outel)
                out.push_back(outel);
        }

        return out;
    }


    void removeChild(ElementBase::Ptr child)
    {
        childs_.erase(std::remove(childs_.begin(), childs_.end(), child), childs_.end());

        // we are not parent of this child anymore
        child->setParent(NULL);
    }

    void setParent(ElementBase::Ptr parent)
    {
        parent_ = parent;
    }

    ElementBase::Ptr getParent() const
    {
        return parent_;
    }


protected:
    bool modified_;
    UniversalUniqueID universal_id_;


    std::string name_;

    std::vector<ElementBase::Ptr> childs_;

    ElementBase::Ptr parent_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, std::uint32_t const version)
    {
        ar(cereal::base_class<ElementWithVariantProperties>(this),
           CEREAL_NVP(modified_), CEREAL_NVP(universal_id_));

           ar(CEREAL_NVP(name_));
           ar(CEREAL_NVP(childs_));
           //we do not save the parent!


    }

    // SerializableInterface interface
public:
    virtual bool isSerializable() const
    {
        return true;
    }
    virtual bool isAsciiSerializable() const
    {
        return false;
    }

    // ElementWithVariantProperties interface
public:
    virtual bool hasVariantProperties()
    {
        return true;
    }
};






} // end nspace

#endif // ELEMENT_BASE_H
