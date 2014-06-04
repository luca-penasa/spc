#ifndef SPC_CORE_POINT_DATA_H
#define SPC_CORE_POINT_DATA_H

#include <boost/any.hpp>
#include <spc/elements/macros.h>
#include <map>
#include <sstream>
#include <spc/elements/ElementBase.h>
#include <cereal/types/map.hpp>
#include <cereal/types/boost_variant.hpp>
#include <boost/variant.hpp>

#include <spc/elements/Sample.h>
namespace spc
{

class CorePoint: public Sample
{
public:


    SPC_OBJECT(CorePoint)

    //!empty const
    CorePoint()
    {
    }

    template <typename T> void setValue(const std::string name, const T &value)
    {
        this->getVariantPropertiesRecord().property(name) = value;
    }

    VariantProperty::VarianT &value(const std::string &name)
    {
        return this->getVariantPropertiesRecord().property(name).value();
    }

    template <typename T> T value(const std::string name) const
    {
        return boost::get<T>(this->getVariantPropertiesRecord().property(name).value());
    }


//    void writeLine(std::ostringstream &stream)
//    {
//        stream << value<float>("distance") << " " << value<float>("intensity")
//               << " " << value<float>("angle") << " "
//               << value<size_t>("cloud_id") << " " << value<size_t>("core_id")
//               << " " << value<size_t>("n_neighbors") << std::endl;
//    }

    bool isValid() const
    {
        return (std::isfinite(value<float>("distance"))
                & std::isfinite(value<float>("intensity"))
                & std::isfinite(value<float>("angle")));
    }

private:

    friend class cereal::access;

    template <class Archive> void sserialize(Archive &ar)
    {
        ar(cereal::base_class<spc::Sample>(this));
    }

};

// streaming for CorePointData
//std::ostream &operator<<(std::ostream &os, const CorePoint &obj);

template <typename T>
std::vector<T> extractPropertyAsVector(const std::vector<CorePoint::Ptr> db,
                                       const std::string prop_name)
{
    std::vector<T> out;
    spcForEachMacro(CorePoint::Ptr entry, db)
    out.push_back(entry->value<T>(prop_name));

    return out;
}

} // end nspace

#endif
