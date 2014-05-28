#ifndef SPC_CORE_POINT_DATA_H
#define SPC_CORE_POINT_DATA_H

#include <boost/any.hpp>
#include <spc/elements/macros.h>
#include <map>
#include <sstream>

namespace spc
{

class CorePointData
{
public:
    typedef  std::map<std::string, boost::any> DataHolderType;
    typedef  std::pair<std::string, boost::any> PairType;

    SPC_OBJECT(CorePointData)

    //!empty const
    CorePointData() {}
//    }

    template <typename T> void setValue (const std::string name, const T &value)
    {
        datadb[name] = value;
    }

    boost::any & value(const std::string &name)
    {
        return datadb[name];
    }

    template <typename T> T value (const std::string name) const
    {
        return boost::any_cast<T> (datadb.at(name));
    }



    DataHolderType getDB()
    {
        return datadb;
    }

    DataHolderType getDB() const //const version
    {
        return datadb;
    }

    void writeLine(std::ostringstream &stream)
    {
        stream    << value<float>("distance") << " "
                  << value<float>("intensity") << " "
                  << value<float>("angle") << " "
                  << value<size_t>("cloud_id") << " "
                  << value<size_t>("core_id") << " "
                  << value<size_t>("n_neighbors") << std::endl;
    }


    bool isValid() const
    {
        return ( std::isfinite(value<float>("distance") ) &
                 std::isfinite(value<float>("intensity") ) &
                 std::isfinite(value<float>("angle") ));


    }



private:
    DataHolderType datadb;


};



// streaming for CorePointData
std::ostream& operator<<(std::ostream& os, const CorePointData& obj);

template <typename T>
std::vector<T> extractPropertyAsVector(const std::vector<CorePointData::Ptr> db, const std::string prop_name)
{
    std::vector<T> out;
    spcForEachMacro(CorePointData::Ptr entry, db)
            out.push_back(entry->value<T>(prop_name));

    return out;
}

}//end nspace


#endif
