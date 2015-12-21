#include "SelectionRubberband.h"

#include <cereal/access.hpp>
#include <spc/core/spc_cereal.hpp>


namespace spc
{

DtiClassType SelectionRubberband::Type ("SelectionRubberband", &ElementBase::Type);

SelectionRubberband::SelectionRubberband()
{
    DLOG(INFO) << "now SelectionRubberband created from def const";
}

SelectionRubberband::SelectionRubberband(const SelectionRubberband &other): ElementBase(other), SelectionOfPointsBase(other)
{
    verts_ = other.verts_;
    verts_2d_ = other.verts_2d_;
    proj_plane_ = other.proj_plane_;
    max_distance_ = other.max_distance_;
    transform_ = other.transform_;
}

SelectionRubberband::SelectionRubberband(const PointCloudXYZBase &verts, float max_distance): max_distance_(max_distance)
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

void spc::SelectionRubberband::updateProjectionPlane()
{
    DLOG(INFO) << "Updating projection plane.";
    proj_plane_.positionFromCentroid(verts_);

    DLOG(INFO) << "Centroid is " << proj_plane_.getPosition().transpose();
    Vector3D n;
    n.normalFromBestFit(verts_);
    proj_plane_.setNormal(n.getNormal());
    LOG(INFO) << "Prjection plane normal " <<  n.getNormal().transpose();
}

bool SelectionRubberband::isPointInPoly(const Vector2f &P, const PolyLine2D &polyVertices) const
{
    int nvert = polyVertices.getNumberOfPoints();
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (((polyVertices.getPoint(i)(1) > P(1)) != (polyVertices.getPoint(j)(1) > P(1)))
                && (P(0) < (polyVertices.getPoint(j)(0) - polyVertices.getPoint(i)(0))
                    * (P(1) - polyVertices.getPoint(i)(1))
                    / (polyVertices.getPoint(j)(1) - polyVertices.getPoint(i)(1))
                    + polyVertices.getPoint(i)(0)))
            c = !c;
    }
    return (bool) c;
}

void SelectionRubberband::updatePolyVertices()
{

    DLOG(INFO) << "Updating poly vertices";

    DLOG(INFO) << "N initial poly verts: " << verts_.getNumberOfPoints();

    updateTMatrix();

    PolyLine3D proj = verts_.transform<PolyLine3D>(transform_);


    DLOG(INFO) << "Cloud projected! copying as 2d cloud";
    verts_2d_.resize(proj.getNumberOfPoints());
    for(int i = 0; i < proj.getNumberOfPoints(); ++i)
    {
        verts_2d_.setPoint(i, proj.getPoint(i).head(2));
    }

    DLOG(INFO) << "there are "<< verts_2d_.getNumberOfPoints() <<  " poly verts";
}

//SelectionRubberband::SelectionRubberband() : max_distance_(1.0)
//{
//}




} // end nspace


//spc::SelectionRubberband test;
//CEREAL_REGISTER_DYNAMIC_INIT(spc)




SPC_CEREAL_REGISTER_TYPE(spc::SelectionRubberband)

CEREAL_CLASS_VERSION( spc::SelectionRubberband, 1 )


