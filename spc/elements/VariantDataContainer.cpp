#include "VariantDataContainer.h"

#include <boost/lexical_cast.hpp>
#include <sstream>


namespace  spc
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


std::ostream& operator<<(std::ostream& os, const spcVariant& obj)
{
    return os << obj.asString().c_str();
}





spcVariant::spcVariant(const spcVariant::VarianT value)
{
    data_ = value;
}

spcVariant::VarianT &spcVariant::value()
{
    return data_;
}

std::string spcVariant::asString() const
{
    to_string_visitor vis;
    boost::apply_visitor(vis, data_);
    return vis.str;
}




spcVariant &spcVariant::operator = (const double &data)
{
    data_ = VarianT(static_cast<float>(data));
    return *this;
}



bool spcVariant::operator == (const spcVariant & other) const
{
    return (other.data_ == data_);
}

} //end nspace
