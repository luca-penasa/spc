#ifndef UNORDERED_DATA_TABLE_H
#define UNORDERED_DATA_TABLE_H

#include <spc/elements/spcObject.h>
#include <Eigen/Dense>

#include <unordered_map>
#include <boost/variant.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/unordered_map.hpp>

namespace spc
{
struct to_string_visitor : boost::static_visitor<>
{
    std::string str;

    template <typename T>
    void operator()(T const& item);

    void operator()(Eigen::Vector3f const & v);

    void operator()(std::string const & s);
};




class Property: public spcObject
{
public:
    SPC_OBJECT(Property)

    // ALLOWED TYPES!
    typedef boost::variant<int, float, std::string, Eigen::Vector3f>  VarianT;

    Property()
    {
    }

    Property(const VarianT value);

    template <typename T>
    Property &operator = (const T &data)
    {
        data_ = VarianT(data);
        return *this;
    }

    Property &operator = (const double &data);

    bool operator == (const Property & other) const;

    VarianT &value ();

    std::string asString() const;

    friend std::ostream& operator<<(std::ostream& os, const Property& obj);

private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(  cereal::base_class<spc::spcObject>( this ),
             CEREAL_NVP(data_));
    }

protected:
    VarianT data_;

};




class UnorderedDataRecord: public spcObject
{
public:
    SPC_OBJECT(UnorderedDataRecord)

    typedef std::pair<std::string, Property> pairT;
    typedef std::unordered_map<std::string, Property> mapT;

    UnorderedDataRecord() {}

    bool operator ==  (const UnorderedDataRecord & other)
    {
        return (properties_ == properties_);
    }

    size_t size() const;

    std::vector<std::string> getKeysList() const;

    void clear();

    bool hasPropertyWithName(const std::string &name) const;

    // this will create a new one if your prop does not exists
    Property & property(const std::string & name);

    // a const one - YOU ARE RESPONSABLE FOR THE EXISTENCE OF SUCH A KEY
    Property property(const std::string & name) const;

    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const UnorderedDataRecord& obj);


private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(  cereal::base_class<spc::spcObject>( this ),
             CEREAL_NVP(properties_));
    }

protected:

    // this will grant me unique names
    mapT properties_;
};



//class UnorderedDataDB: public spcObject
//{
//    SPC_OBJECT(UnorderedDataDB)
//public:
//    UnorderedDataDB() {}

//    UnorderedDataRecord::Ptr record(const boost::uuids::uuid &uuid)
//    {
//        spcForEachMacro(UnorderedDataRecord::Ptr p , records_)
//        {
//            if (p->getUUID() == uuid)
//                return p;
//        }

//        UnorderedDataRecord::Ptr new_p (new UnorderedDataRecord(uuid));
//        return new_p;
//    }

//    std::set<UnorderedDataRecord::Ptr> records_;
//};

}//end nspace
#endif
