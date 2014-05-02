#ifndef SPC_CORE_POINT_DATA_H
#define SPC_CORE_POINT_DATA_H

#include <spc/common/common_includes.h>

#include <boost/any.hpp>



namespace spc
{

class CorePointData
{
public:
    typedef  std::map<std::string, boost::any> DataHolderType;
    typedef  std::pair<std::string, boost::any> PairType;

    typedef boost::shared_ptr<CorePointData> Ptr;
    typedef const boost::shared_ptr<CorePointData> ConstPtr;

    //!empty const
    CorePointData() {}

    boost::any & value(const std::string name)
    {
        return datadb[name];
    }

    boost::any value(const std::string name) const
    {
        return datadb.at(name);
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
        stream    << boost::any_cast<float> (this->value("distance")) << " "
                  << boost::any_cast<float> (this->value("intensity")) << " "
                  << boost::any_cast<float> (this->value("angle")) << " "
                  << boost::any_cast<size_t> (this->value("cloud_id")) << " "
                  << boost::any_cast<size_t> (this->value("core_point_id")) << " "
                  << boost::any_cast<size_t> (this->value("n_neighbors")) << std::endl;
    }

    bool isValid() const
    {
        return ( std::isfinite(boost::any_cast<float> (this->value("distance")) ) &
                 std::isfinite(boost::any_cast<float> (this->value("intensity")) ) &
                 std::isfinite(boost::any_cast<float> (this->value("angle")) ));


    }



private:
    DataHolderType datadb;


};



// streaming for CorePointData
std::ostream& operator<<(std::ostream& os, const CorePointData& obj);

// easy accessor
std::vector<size_t> extractPropertyAsVector(const std::vector<CorePointData::Ptr> db, const std::string prop_name);

}//end nspace


#endif
