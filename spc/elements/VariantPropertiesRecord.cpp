#include <spc/elements/VariantPropertiesRecord.h>
#include <boost/lexical_cast.hpp>

#include <spc/elements/ElementBase.h>
#include <boost/variant.hpp>
#include <boost/uuid/uuid.hpp>

#include <unordered_map>
#include <iostream>

namespace spc
{

std::ostream &operator<<(std::ostream &os, const VariantPropertiesRecord &obj)
{
    std::vector<std::string> keys = obj.getKeysList();

    size_t counter = 0;
    size_t n = keys.size() - 1;

    spcForEachMacro(std::string k, keys)
    {
        VariantProperty p = obj.property(k);
        os << k << ": " << p << ";";
        if (counter != n)
            os << " ";

        counter++;
    }

    return os;
}

size_t VariantPropertiesRecord::size() const
{
    return properties_.size();
}

std::vector<std::string> VariantPropertiesRecord::getKeysList() const
{
    std::vector<std::string> l;
    spcForEachMacro(pairT el, properties_)
    l.push_back(el.first);

    return l;
}

void VariantPropertiesRecord::clear()
{
    properties_.clear();
}

bool VariantPropertiesRecord::hasPropertyWithName(const std::string &name) const
{
    spcForEachMacro(pairT el, properties_)
    {
        if (el.first == name)
            return true;
    }
    return false;
}

VariantProperty VariantPropertiesRecord::property(const std::string &name) const
{
    if (this->hasPropertyWithName(name))
        return properties_.at(name);
    else
        return VariantProperty();
}

VariantProperty &VariantPropertiesRecord::property(const std::string &name)
{
    return properties_[name];
}

std::string VariantPropertiesRecord::toString() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

} // end nspace
