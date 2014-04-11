#ifndef ORIENTATION_H
#define ORIENTATION_H


#include <spc/elements/plane.h>
#include <iostream>

#include <spc/elements/movable_element.h>
#include <fstream>
namespace spc
{
///
/// \brief an spcAttitude is a more sophisticated way to rapresent a plane in space
/// It is actually the same thing of a Plane object with a position in space that localize the measure
/// \note Typical notation for an attitude is 10/245 with 10 dip angle (0-90) and 245 is azimut from N (0-360) - the dip.
class spcAttitude: public spcPlane
{
public:

    typedef boost::shared_ptr<spcAttitude> Ptr;
    typedef boost::shared_ptr<const spcAttitude> ConstPtr;



    /// def contructor
    spcAttitude();

    ///
    /// \brief spcAttitude
    /// \param direction is the normal of the plane
    /// \param position is the position in space for this attitude measure
    ///
    spcAttitude(const Vector3f &direction, const Vector3f &position);

    ///
    /// \brief spcAttitude constructor (geologic-aware)
    /// \param dip is the azimutal angle of dip direction with the north [0-360]
    /// \param dipAngle formed with the orizontal plane
    ///
    spcAttitude(const float dipAngle, const float dip, Vector3f position = Vector3f::Zero());


    //copy const
    spcAttitude(const spcAttitude & att): spcPlane(att) {}


    //////////////////////////////////////
    //// GEOLOGICAL AWARE GETTERS ////////
    //////////////////////////////////////

    /// Dip direction is the projection of this plane's normal on the
    /// horizontal plane. Should be considered a line, so sign is not really important
    /// z componentes is zero, but we keep the 3d size of vector
    /// is normalized
    Vector3f getDipDirectionVector() const;

    /// strike is the cross product of the Dip direction with the normal
    /// note that a rake is only a line. In this case we represent it as a vector
    /// but the direction can be inverted as you like
    /// is a normalized vector
    Vector3f getStrikeVector() const;

    /// the dip vector is the projection of the dip direction on the
    /// plane itself, is normalized
    Vector3f getDipVector() const;

    /// dip is the angle formed by the dipVector with the horizontal plane
    /// here represented by the dipDirection
    /// can have only positive values and is in degrees - NOT radians -
    /// is always in [0, 90]
    float getDipAngle() const;

    /// is the angle formed by the dipDirectionVector with the North!
    float getDip() const;

    ///format dip and dip angle as astring
    std::string getDipAndDipAngleAsString() const;

protected:

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
//        ar & boost::serialization::make_nvp("position",boost::serialization::base_object<spcMovableElement >(*this));
//        ar & boost::serialization::make_nvp("normal", normal_);


        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(spcPlane);
    }



};

/// print out as stream
std::ostream& operator<<(std::ostream& os, const spcAttitude& obj);

}//end nspace

#endif // ORIENTATION_H
