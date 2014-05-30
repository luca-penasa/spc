#include <spc/elements/Plane.h>

using namespace Eigen;
namespace spc
{

Transform<float, 3, Affine, AutoAlign> Plane::get2DArbitraryRefSystem() const
{
    Transform<float, 3, Affine, AutoAlign> T;

    Matrix3f new_basis;
    new_basis.setIdentity();

    //    T.setIdentity();
    Eigen::Vector3f n = normal_.getUnitNormal();

    float minval = n.array().abs().minCoeff();

    /// seek for a good unit axis to project on plane
    /// and then use it as X direction of the 2d local system
    Vector3f proj_axis;
    int min_id;
    for (int i = 0; i < 3; ++i) {
        if (minval == n.array().abs()(i))
            min_id = i;
    }

    // we are at the minimum
    if (min_id == 0) // on x
        proj_axis = Vector3f::UnitX();
    else if (min_id == 1)
        proj_axis = Vector3f::UnitY();
    else
        proj_axis = Vector3f::UnitZ();

    // project the selected axis on the plane
    Vector3f second_ax = projectOnPlane(proj_axis);
    second_ax.normalize();

    Translation3f translation(-getPosition());

    //    T = T * translation ;
    new_basis.col(0) = n;
    new_basis.col(1) = second_ax;
    new_basis.col(2) = n.cross(second_ax);

    Matrix3f Rot;
    Rot = Quaternionf().setFromTwoVectors(n, second_ax);
    //    T.matrix().col(3).head(3) = -getPosition();

    //    AngleAxis<float> R = new_basis;
    T = Rot * translation;

    std::cout << "origin:" << getPosition() << std::endl;
    // generate a suitable ref frame

    return T;
}

} // end nspace
