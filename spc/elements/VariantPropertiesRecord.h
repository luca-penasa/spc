#ifndef UNORDERED_DATA_TABLE_H
#define UNORDERED_DATA_TABLE_H

#include <spc/methods/spc_eigen.h>

#include <unordered_map>
#include <boost/variant.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/unordered_map.hpp>

#include <spc/elements/VariantProperty.h>
namespace spc
{

class VariantPropertiesRecord
{
public:
    SPC_OBJECT(VariantPropertiesRecord)
EXPOSE_TYPE
    typedef std::pair<std::string, VariantProperty> pairT;
    typedef std::unordered_map<std::string, VariantProperty> mapT;

    VariantPropertiesRecord()
    {
    }

    bool operator==(const VariantPropertiesRecord &other)
    {
        return (properties_ == other.properties_);
    }

    bool operator!=(const VariantPropertiesRecord &other)
    {
        return !(properties_ == other.properties_);
    }

    size_t size() const;

    std::vector<std::string> getKeysList() const;

    void clear();

    bool hasPropertyWithName(const std::string &name) const;

    // this will create a new one if your prop does not exists
    VariantProperty &property(const std::string &name);

    // a const one - YOU ARE RESPONSABLE FOR THE EXISTENCE OF SUCH A KEY
    VariantProperty property(const std::string &name) const;

    std::string toString() const;

    friend std::ostream &operator<<(std::ostream &os,
                                    const VariantPropertiesRecord &obj);

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(properties_));
    }

protected:
    // this will grant me unique names
    mapT properties_;
};

} // end nspace
#endif
