#include <spc/elements/plane.h>

namespace spc
{

Plane::Plane() : normal_(0,0,1), distance_(0.0f)
{

}


Plane::Plane(const Vector4f parameters)
{
      setPlaneParameters(parameters);
}

Plane::Plane(const Vector3f n, const float d)
{
    setUnitNormal(n);
    setD(d);
}

}//end nspace
