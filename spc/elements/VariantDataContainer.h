#ifndef SPC_VARIANT_H
#define SPC_VARIANT_H

#include <spc/elements/ElementBase.h>
#include <Eigen/Dense>
#include <boost/variant.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/unordered_map.hpp>

namespace spc
{

struct to_string_visitor : boost::static_visitor<>
{
    std::string str;

    template <typename T> void operator()(T const &item);

    void operator()(Eigen::Vector3f const &v);

    void operator()(std::string const &s);
};

class VariantDataContainer
{
public:
    SPC_OBJECT(VariantDataContainer)

    // ALLOWED TYPES!
    typedef boost::variant<int, float, std::string, Eigen::Vector3f> VarianT;

    VariantDataContainer()
    {
    }

    VariantDataContainer(const VarianT value);

    template <typename T> VariantDataContainer &operator=(const T &data)
    {
        data_ = VarianT(data);
        return *this;
    }

    VariantDataContainer &operator=(const double &data);

    bool operator==(const VariantDataContainer &other) const;

    VarianT &value();

    std::string asString() const;

    friend std::ostream &operator<<(std::ostream &os, const VariantDataContainer &obj);

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(data_));
    }

protected:
    VarianT data_;
};

} // end nspace

#endif
