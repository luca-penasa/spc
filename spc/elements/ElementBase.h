#pragma once
#ifndef SPC_OBJECT_H
#define SPC_OBJECT_H
#include <spc/core/spc_eigen.h>
#include <spc/core/macros.h>
#include <spc/elements/SerializableInterface.h>
#include <spc/core/logging.h>
#include <cereal/cereal.hpp>



namespace spc
{


class ElementBase : public ISerializable,
        public std::enable_shared_from_this<ElementBase>
{
public:
    spcTypedefSharedPtrs(ElementBase)
    EXPOSE_TYPE_BASE




    ElementBase()
    {
    }



    Ptr getPtr();

    ElementBase (const ElementBase& other)
    {

    }

    virtual ~ElementBase()
    {

    }

	virtual ElementBase::Ptr clone() const = 0;

    std::string getElementName() const
    {
        return name_;
    }

    std::string getClassNameFromDtiClass() const
    {
        return this->getType()->getClassName();
    }

    void setElementName(const std::string &name)
    {
        name_ = name;
    }

    //!\todo this could go in a separated interface
    void addChild(ElementBase::Ptr child);

    std::vector<ElementBase::Ptr> getChilds() const
    {
        return childs_;
    }

    //! only at the first level
    std::vector<ElementBase::Ptr> getChildsThatAre(const DtiClassType * dti) const;

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


    std::vector<ElementBase::Ptr> findElementsThatAre(const DtiClassType * dti);


    std::vector<ElementBase::Ptr> findElementsInParents(const DtiClassType * dti) const;


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


    void removeChild(ElementBase::Ptr child);

    void setParent(const ElementBase::Ptr parent);

    ElementBase::Ptr getParent() const
    {
        return parent_;
    }


protected:
    std::string name_;

    std::vector<ElementBase::Ptr> childs_;

    ElementBase::Ptr parent_;

private:
    friend class cereal::access;

	template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(CEREAL_NVP(name_),
           CEREAL_NVP(childs_)
           );
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


};



} // end nspace

CEREAL_CLASS_VERSION(spc::ElementBase, 1)


#endif // ELEMENT_BASE_H
