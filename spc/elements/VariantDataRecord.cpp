#include <spc/elements/VariantDataRecord.h>
#include <boost/lexical_cast.hpp>

#include <spc/elements/ElementBase.h>
#include <boost/variant.hpp>
#include <boost/uuid/uuid.hpp>

#include <unordered_map>
#include <iostream>

namespace spc
{

std::ostream& operator<<(std::ostream& os, const VariantDataRecord& obj)
{
    std::vector<std::string> keys = obj.getKeysList();

    size_t counter = 0;
    size_t n = keys.size() -1;

    spcForEachMacro(std::string k, keys)
    {
        spcVariant p =obj.property(k);
        os << k << ": " << p << ";";
        if (counter != n)
            os << " " ;

        counter++;
    }

    return os;
}


size_t VariantDataRecord::size() const
{
    return properties_.size();
}

std::vector<std::string> VariantDataRecord::getKeysList() const
{
    std::vector<std::string> l;
    spcForEachMacro(pairT el, properties_)
            l.push_back(el.first);

    return l;
}

void VariantDataRecord::clear()
{
    properties_.clear();
}

bool VariantDataRecord::hasPropertyWithName(const std::string &name) const
{
    spcForEachMacro(pairT el, properties_)
    {
        if (el.first == name)
            return true;
    }
    return false;
}

spcVariant VariantDataRecord::property(const std::string &name) const
{
    if (this->hasPropertyWithName(name))
        return properties_.at(name);
}

std::string VariantDataRecord::toString() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

spcVariant &VariantDataRecord::property(const std::string &name)
{
    return properties_[name];
}

}//end nspace
