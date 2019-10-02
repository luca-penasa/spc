#ifndef SPC_VARIANT_H
#define SPC_VARIANT_H

#include <spc/core/macros.h>
#include <spc/core/spc_eigen.h>
#include <boost/variant.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/unordered_map.hpp>
namespace spc
{

struct to_string_visitor : boost::static_visitor<>
{
    std::string str;

    template <typename T> void operator()(T const &item);

	template <typename T>
	void operator()(std::vector<T> const v);

    void operator()(Eigen::Vector3f const &v);

    void operator()(std::string const &s);
};

class VariantProperty
{
public:


    // ALLOWED TYPES!
    typedef boost::variant<int, float, std::string> VarianT;

    VariantProperty()
    {
    }

    VariantProperty(const VarianT &value);

    template <typename T> VariantProperty &operator=(const T &data)
    {
        data_ = VarianT(data);
        return *this;
    }

    VariantProperty &operator=(const double &data);

    bool operator==(const VariantProperty &other) const;

    VarianT &value();

    std::string asString() const;

    friend std::ostream &operator<<(std::ostream &os,
                                    const VariantProperty &obj);

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
