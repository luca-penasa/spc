#include "VariantProperty.h"

#include <boost/lexical_cast.hpp>
#include <sstream>

namespace spc
{

template <typename T> void
to_string_visitor::operator()(T const &item)
{
    str = boost::lexical_cast<std::string>(item);
}

void to_string_visitor::operator()(Eigen::Vector3f const &v)
{
    std::stringstream stream;
    stream << "[" << v(0) << " " << v(1) << " " << v(2) << "]";

    str = stream.str();
}

void to_string_visitor::operator()(std::string const &s)
{
    std::stringstream stream;
    stream << "\"" << s.c_str() << "\"";

    str = stream.str();
}

template <typename T>
void to_string_visitor::operator()(std::vector<T> const v)
{
    std::stringstream stream;
    stream << "[" ;
    for (int i = 0 ; i < v.size(); ++i)
    {
        T el = v.at(i);
        stream << boost::lexical_cast<std::string>(el);
        if (i != v.size() - 1)
            stream  << "; ";

    }
    stream << "]";

    str = stream.str();
}

std::ostream &operator<<(std::ostream &os, const VariantProperty &obj)
{
    return os << obj.asString().c_str();
}

VariantProperty::VariantProperty(const VarianT &value)
{
    data_ = value;
}

VariantProperty::VarianT &VariantProperty::value()
{
    return data_;
}

std::string VariantProperty::asString() const
{
    to_string_visitor vis;
    boost::apply_visitor(vis, data_);
    return vis.str;
}

VariantProperty &VariantProperty::operator=(const double &data)
{
    data_ = VarianT(static_cast<float>(data));
    return *this;
}

bool VariantProperty::operator==(const VariantProperty &other) const
{
    return (other.data_ == data_);
}

} // end nspace
