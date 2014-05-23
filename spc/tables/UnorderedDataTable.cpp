#include <spc/tables/UnorderedDataTable.h>
#include <boost/lexical_cast.hpp>

#include <spc/elements/spcObject.h>
#include <boost/variant.hpp>
#include <boost/uuid/uuid.hpp>

#include <spc/elements/UniversalUniqueObject.h>
#include <unordered_map>
#include <iostream>

namespace spc
{

template <typename T>
void to_string_visitor::operator()(T const& item)
{
    str = boost::lexical_cast<std::string>(item);
}

void to_string_visitor::operator()(Eigen::Vector3f const & v)
{
    std::stringstream stream;
    stream << "["<< v(0) << " " << v(1) << " " << v(2) << "]";

    str = stream.str();
}

void to_string_visitor::operator()(std::string const & s)
{
    std::stringstream stream;
    stream << "\""<< s.c_str() <<"\"";

    str = stream.str();
}


std::ostream& operator<<(std::ostream& os, const Property& obj)
{
    return os << obj.asString().c_str();
}

std::ostream& operator<<(std::ostream& os, const UnorderedDataRecord& obj)
{
    std::vector<std::string> keys = obj.getKeysList();

    size_t counter = 0;
    size_t n = keys.size() -1;

    spcForEachMacro(std::string k, keys)
    {
        Property p =obj.property(k);
        os << k << ": " << p << ";";
        if (counter != n)
            os << " " ;

        counter++;
    }

    return os;
}



Property::Property(const Property::VarianT value)
{
    data_ = value;
}

Property::VarianT &Property::value()
{
    return data_;
}

std::string Property::asString() const
{
    to_string_visitor vis;
    boost::apply_visitor(vis, data_);
    return vis.str;
}




Property &Property::operator = (const double &data)
{
    data_ = VarianT(static_cast<float>(data));
    return *this;
}



bool Property::operator == (const Property & other) const
{
    return (other.data_ == this->data_);
}

size_t UnorderedDataRecord::size() const
{
    return properties_.size();
}

std::vector<std::string> UnorderedDataRecord::getKeysList() const
{
    std::vector<std::string> l;
    spcForEachMacro(pairT el, properties_)
            l.push_back(el.first);

    return l;
}

void UnorderedDataRecord::clear()
{
    properties_.clear();
}

bool UnorderedDataRecord::hasPropertyWithName(const std::string &name) const
{
    spcForEachMacro(pairT el, properties_)
    {
        if (el.first == name)
            return true;
    }
    return false;
}

Property UnorderedDataRecord::property(const std::string &name) const
{
    return properties_.at(name);
}

std::string UnorderedDataRecord::toString() const
{
    std::stringstream stream;
    stream << *this;
    return stream.str();
}

Property &UnorderedDataRecord::property(const std::string &name)
{
    return properties_[name];
}

}//end nspace
