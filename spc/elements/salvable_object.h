#ifndef SALVABLE_OBJECT_H
#define SALVABLE_OBJECT_H

#include <strstream>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <Eigen/Dense>

#include <boost/serialization/export.hpp>
//#include <boost/serialization/split_member.hpp>
#include<boost/serialization/split_free.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>




/// teach boost how to serialize eigen matrixes
/// from http://stackoverflow.com/questions/12851126/serializing-eigens-matrix-using-boost-serialization
namespace boost
{

template<class Archive>
inline void serialize(
        Archive & ar,
        Eigen::Vector3f& t,
        const unsigned int file_version
        )
{
    ar & boost::serialization::make_nvp("vector3d_x", t(0));
    ar & boost::serialization::make_nvp("vector3d_y", t(1));
    ar & boost::serialization::make_nvp("vector3d_z", t(2));
}

template<class Archive>
inline void serialize(
        Archive & ar,
        pcl::PointXYZ& t,
        const unsigned int file_version
        )
{
    ar & boost::serialization::make_nvp("px", t.x);
    ar & boost::serialization::make_nvp("py", t.y);
    ar & boost::serialization::make_nvp("pz", t.z);
}

namespace serialization {

template<class Archive>
inline void load(
        Archive & ar,
        pcl::PointCloud<pcl::PointXYZ>& t,
        const unsigned int file_version
        )
{
    int n;
    ar >> boost::serialization::make_nvp("size", n);

    t.resize(n);

    for (int i = 0; i < n; ++i)
    {
        ar >> boost::serialization::make_nvp("point3d", t.at(i));
    }


}

template<class Archive>
inline void save(
        Archive & ar,
        const pcl::PointCloud<pcl::PointXYZ>& t,
        const unsigned int file_version
        )
{
    int n = t.size();

    ar & boost::serialization::make_nvp("size", n);


    for (int i = 0; i < n; ++i)
    {
        ar & boost::serialization::make_nvp("point3d", t.at(i));
    }



}



} //end nspace serialization
}//end nspace boost


BOOST_SERIALIZATION_SPLIT_FREE(pcl::PointCloud<pcl::PointXYZ>)


namespace spc
{


class spcSerializableObject
{
public:
    typedef boost::shared_ptr<spcSerializableObject> Ptr;
    typedef boost::shared_ptr<const spcSerializableObject> ConstPtr;


public:
    spcSerializableObject();

    virtual bool isSPCSerializable() {return true;}

protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        //nothing
    }


};









}//end nspace



#endif // SALVABLE_OBJECT_H
