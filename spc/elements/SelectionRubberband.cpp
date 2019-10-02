#include "SelectionRubberband.h"

#include <cereal/access.hpp>
#include <spc/core/spc_cereal.hpp>

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/Plane.h>
//#include <pcl/io/pcd_io.h> //for debug only
#include <spc/elements/PointCloudPcl.h>

#include <boost/foreach.hpp>

//#include <spc/elements/PointCloudSpc.h>

#include <spc/elements/StratigraphicModelBase.h>

#include <cereal/cereal.hpp>

#include <cereal/access.hpp>
#include <spc/elements/Plane.h>

namespace spc {

DtiClassType SelectionRubberband::Type("SelectionRubberband", &GeometricElement3DBase::Type);

SelectionRubberband::SelectionRubberband()
    : proj_plane_(new Plane)
{
    DLOG(INFO) << "now SelectionRubberband created from def const";
}

SelectionRubberband::SelectionRubberband(const SelectionRubberband& other)
    : GeometricElement3DBase(other)
    , SelectionOfPointsBase(other)
    , proj_plane_(new Plane)
{
    verts_ = other.verts_;
    verts_2d_ = other.verts_2d_;
    *proj_plane_ = *other.proj_plane_;
    max_distance_ = other.max_distance_;
    transform_ = other.transform_;
}

SelectionRubberband::SelectionRubberband(const PointCloudXYZBase& verts, float max_distance)
    : max_distance_(max_distance)
    , proj_plane_(new Plane)
{
    DLOG(INFO) << "creating rubberband with  " << verts.getNumberOfPoints() << " number of verts";
    DLOG(INFO) << "max distance set to " << max_distance;
    verts_ = PolyLine3D(verts);

    DLOG(INFO) << "polyline3d created";
    verts_.asEigenMatrix();
    DLOG(INFO) << "aseigen called and its ok";
    updateProjectionPlane();
    updatePolyVertices();
}

PolyLine3D SelectionRubberband::getVertices() const
{
    return verts_;
}

Plane::Ptr SelectionRubberband::getProjectionPlane() const
{
    return proj_plane_;
}

bool SelectionRubberband::contains(const Vector3f& obj) const
{

    DLOG(INFO) << "max distance is " << max_distance_;
    DLOG(INFO) << "distance is " << std::abs(proj_plane_->distanceTo(obj));

    if (std::abs(proj_plane_->distanceTo(obj)) > max_distance_)
    {
        return false;
    }
    else {
        DLOG(INFO) << "checking with polyline";
        Vector2f on_plane = (transform_ * obj).head(2);
        bool status = isPointInPoly2(on_plane, verts_2d_);
        DLOG(INFO) << "status is "<< status;
        return status;
    }
}

void spc::SelectionRubberband::updateProjectionPlane()
{
    DLOG(INFO) << "Updating projection plane.";
    proj_plane_->positionFromCentroid(verts_);

    DLOG(INFO) << "Centroid is " << proj_plane_->getPosition().transpose();
    Vector3D n;
    n.normalFromBestFit(verts_);
    proj_plane_->setNormal(n.getNormal());
    LOG(INFO) << "Projection plane normal " << n.getNormal().transpose();
}

void SelectionRubberband::updateTMatrix()
{
    transform_ = proj_plane_->get2DArbitraryRefSystem();
    DLOG(INFO) << "transform is " << transform_.matrix();
}

void SelectionRubberband::updatePolyVertices()
{

    DLOG(INFO) << "Updating poly vertices";

    DLOG(INFO) << "N initial poly verts: " << verts_.getNumberOfPoints();

    updateTMatrix();

    PolyLine3D proj = verts_.transform<PolyLine3D>(transform_);

    DLOG(INFO) << "Cloud projected! copying as 2d cloud";
    verts_2d_.resize(proj.getNumberOfPoints());
    for (int i = 0; i < proj.getNumberOfPoints(); ++i) {
        verts_2d_.setPoint(i, proj.getPoint(i).head(2));
    }

    DLOG(INFO) << "there are " << verts_2d_.getNumberOfPoints() << " poly verts";
}

void SelectionRubberband::linkToStratigraphicModel(StratigraphicModelBasePtr mod)
{
    if (hasModel()) {
        LOG(WARNING) << "linked model changed";
    }
    model_ = mod;
}

StratigraphicModelBasePtr SelectionRubberband::getLinkedStratigraphicModel() const
{
    return model_;
}

} // end nspace

SPC_CEREAL_REGISTER_TYPE(spc::SelectionRubberband)


void spc::SelectionRubberband::applyTransform(const GeometricElement3DBase::TransformT &transform)
{
    verts_ = verts_.transform<spc::PolyLine3D>(transform);
    updateProjectionPlane();
    updatePolyVertices();
}
