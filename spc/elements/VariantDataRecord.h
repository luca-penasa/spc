#ifndef UNORDERED_DATA_TABLE_H
#define UNORDERED_DATA_TABLE_H

#include <spc/elements/ElementBase.h>
#include <Eigen/Dense>

#include <unordered_map>
#include <boost/variant.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/unordered_map.hpp>

#include <spc/elements/VariantDataContainer.h>
namespace spc
{

class VariantDataRecord: public spcObject
{
public:
    SPC_OBJECT(VariantDataRecord)

    typedef std::pair<std::string, spcVariant> pairT;
    typedef std::unordered_map<std::string, spcVariant> mapT;

    VariantDataRecord() {}

    bool operator ==  (const VariantDataRecord & other)
    {
        return (properties_ == other.properties_);
    }

    bool operator != (const VariantDataRecord &other)
    {
        return !(properties_ == other.properties_);
    }

    size_t size() const;

    std::vector<std::string> getKeysList() const;

    void clear();

    bool hasPropertyWithName(const std::string &name) const;

    // this will create a new one if your prop does not exists
    spcVariant & property(const std::string & name);

    // a const one - YOU ARE RESPONSABLE FOR THE EXISTENCE OF SUCH A KEY
    spcVariant property(const std::string & name) const;

    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const VariantDataRecord& obj);



private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(  make_nvp("spcObject", cereal::base_class<spc::spcObject>( this )),
             CEREAL_NVP(properties_));
    }

protected:

    // this will grant me unique names
    mapT properties_;
};



}//end nspace
#endif
