#ifndef ORIENTATION_H
#define ORIENTATION_H


#include <spc/elements/plane.h>
#include <iostream>

#include <spc/elements/point3d.h>

namespace spc
{
///
/// \brief an Attitude is a more sophisticated way to rapresent a plane in space
/// It is actually the same thing of a Plane object with a position in space that localize the measure
/// \note Typical notation for an attitude is 10/245 with 10 dip angle (0-90) and 245 is azimut from N (0-360) - the dip.
class Attitude: public Plane
{
public:

    typedef typename boost::shared_ptr<Attitude> Ptr;


    /// def contructor
    Attitude();

    ///
    /// \brief Attitude
    /// \param direction is the normal of the plane
    /// \param position is the position in space for this attitude measure
    ///
    Attitude(const Vector3f direction, const Vector3f position);

    ///
    /// \brief Attitude constructor (geologic-aware)
    /// \param dip is the azimutal angle of dip direction with the north [0-360]
    /// \param dipAngle formed with the orizontal plane
    ///
    Attitude(const float dipAngle, const float dip, Vector3f position = Vector3f::Zero())
    {

        if (dipAngle <= 0.0 | dipAngle >= 90)
            std::cout << "WARN: dipAngle out of bounds. Are you sure?" << std::endl;

        Vector3f n = Vector3f::UnitZ(); //is a vertical vector
        AngleAxis<float> rot_y (dipAngle / 180 * M_PI, Vector3f::UnitY());
        AngleAxis<float> rot_z (dip / 180*M_PI, -Vector3f::UnitZ());

        Vector3f out_n =  rot_z * rot_y * n;

        normal_ = out_n;
        position_ = position;
    }


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

};

/// print out as stream
std::ostream& operator<<(std::ostream& os, const Attitude& obj);

}//end nspace

#endif // ORIENTATION_H
