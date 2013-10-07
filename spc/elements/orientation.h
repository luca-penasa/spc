#ifndef ORIENTATION_H
#define ORIENTATION_H


#include <spc/elements/plane.h>

namespace spc
{
///
/// \brief The Orientation class provides a more sophisticated way to rapresent an orientation in space
/// It is actually the same of a Plane object with a position in space that localize this measure
///
class Orientation: public Plane
{
public:
    Orientation();

    Orientation(const Vector3f position, const Vector3f direction)
    {
        normal_ = direction;
        position_ = position;
    }

private:
    Vector3f position_;


};

}//end nspace

#endif // ORIENTATION_H
