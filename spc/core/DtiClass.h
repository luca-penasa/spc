#pragma once
#ifndef DTICLASS_H
#define DTICLASS_H

#include <string>

namespace spc {

class DtiClassType {
public:
    DtiClassType(const std::string &name, DtiClassType* parent);

    virtual ~DtiClassType();

    const std::string getClassName() const
    {
        return class_name_;
    }

    bool hasParent() const
    {
        return parent_ != nullptr;
    }

    DtiClassType* getParent() const
    {
        return parent_;
    }

    // says it the two types are the same
    bool operator==(const DtiClassType& other) const
    {
        return (class_name_ == other.class_name_);
    }

    bool isA(const DtiClassType* other) const
    {
        DtiClassType const* start = this;
        while (start) {
            if (*start == *other)
                return true;
            else
                start = start->getParent();
        }
        return false;
    }

    DtiClassType& operator=(const DtiClassType& other)
    {
        this->parent_ = other.getParent();
        this->class_name_ = other.getClassName();
        return *this;
    }

private:
    std::string class_name_;
    DtiClassType* parent_;
};

} // end nspace

#endif // DTICLASS_H
