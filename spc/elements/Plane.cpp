#include <spc/elements/Plane.h>

using namespace Eigen;
namespace spc {

DtiClassType Plane::Type("Plane", &Point3D::Type);

Transform<float, 3, Affine, AutoAlign> Plane::get2DArbitraryRefSystem() const
{
    Eigen::Vector3f n = normal_.getUnitNormal();

    /// seek for a good unit axis to project on plane
    /// and then use it as X direction of the 2d local system
    size_t min_id;
    n.array().abs().minCoeff(&min_id);

    DLOG(INFO) << "min id is " << min_id;
    Vector3f proj_axis = Vector3f::Zero();
    proj_axis(min_id) = 1; // unity on that axis

    // project the selected axis on the plane
    Vector3f second_ax = projectVectorOnPlane(proj_axis);
    second_ax.normalize();

    Vector3f first_ax = n.cross(second_ax);

    first_ax.normalize();

    Transform<float, 3, Affine, AutoAlign> T;
//    T.matrix().fill(0);
    T.matrix().col(0).head(3) = first_ax;
    T.matrix().col(1).head(3) = second_ax;
    T.matrix().col(2).head(3) = n;
//    T.matrix()(3, 3) = 1;


    DLOG(INFO) << "Transform computed \n " << T.inverse().matrix() << "normal was " << n;

    DLOG(INFO) << "In fact T*n " <<T.inverse() *n;
    return T.inverse();
}

} // end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Plane);
