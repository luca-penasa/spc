#ifndef SALVABLE_OBJECT_H
#define SALVABLE_OBJECT_H

#include <strstream>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <Eigen/Dense>


/// teach boost how to serialize eigen matrixes
/// from http://stackoverflow.com/questions/12851126/serializing-eigens-matrix-using-boost-serialization
namespace boost
{
    template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline void serialize(
        Archive & ar,
        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
        const unsigned int file_version
    )
    {
        for(size_t i=0; i<t.size(); i++)
            ar & t.data()[i];
    }
}


namespace spc
{


class spcSalvableObject
{
public:
    spcSalvableObject();

    template <class Archive>
    void serialize(Archive & ar, const unsigned int version) {/*nothing*/}




};


}//end nspace

#endif // SALVABLE_OBJECT_H
