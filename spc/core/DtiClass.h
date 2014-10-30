#pragma once
#ifndef DTICLASS_H
#define DTICLASS_H

#include <string>
#include <iostream> //for debug

class DtiClassType
{
public:

    DtiClassType(std::string name,  DtiClassType * parent = 0);

    virtual ~DtiClassType();

    const std::string getClassName() const
    {
        return class_name_;
    }

    bool hasParent() const
    {
        return parent_;
    }

    DtiClassType *getParent() const
    {
        return parent_;
    }


    // says it the two types are the same
    bool operator==(const DtiClassType &other) const
    {
        return (class_name_ == other.class_name_);
    }

    bool isA(const DtiClassType *other) const
    {
        DtiClassType const *start = this;
        while (start) {
            if (*start == *other)
                return true;
            else
                start = start->getParent();
        }
        return false;
    }

    DtiClassType &operator = (const DtiClassType &other)
    {
        this->parent_ = other.getParent();
        this->class_name_ = other.getClassName();
        return *this;
    }

private:
    std::string class_name_;
    DtiClassType *parent_;


};

#endif // DTICLASS_H
