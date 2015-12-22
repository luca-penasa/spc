#pragma once
#ifndef SPCPLANARSELECTION_H
#define SPCPLANARSELECTION_H

#include <spc/elements/ElementBase.h>
#include <spc/elements/SelectionBase.h>
#include <spc/elements/templated/PolyLine3D.h>

namespace spc {
spcFwdDeclSharedPtr(StratigraphicModelBase)
spcFwdDeclSharedPtr(Plane)

/// NOTE this class must be splitted in a filter and a serializable object
/// it is not good that an object does operations on data. Filters do them.

class SelectionRubberband : public ElementBase, public SelectionOfPointsBase {
public:
    SPC_ELEMENT(SelectionRubberband)
    EXPOSE_TYPE

    typedef Transform<float, 3, Affine, AutoAlign> TransT;

    /** def const */
    SelectionRubberband();

    SelectionRubberband(const SelectionRubberband& other);

    SelectionRubberband(const PointCloudXYZBase& verts, float max_distance = 1);

    PolyLine3D getVertices() const;

    PlanePtr getProjectionPlane() const;

    void setVertices(const PointCloudXYZBase& verts)
    {
        //        verts_ = PolyLine3D(verts);
        updateProjectionPlane();
        updatePolyVertices();
    }

    virtual bool contains(const Vector3f& obj) const override;

    float getMaxDistance() const
    {
        return max_distance_;
    }

    void setMaxDistance(float d)
    {
        max_distance_ = d;
    }

    TransT getPojectionTransform() const
    {
        return transform_;
    }

protected:
    void updateProjectionPlane();

    /// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
    inline bool isPointInPoly(const Eigen::Vector2f& P,
                       const PolyLine2D& polyVertices) const
    {
        int nvert = polyVertices.getNumberOfPoints();
        int i, j, c = 0;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((polyVertices.getPoint(i)(1) > P(1)) != (polyVertices.getPoint(j)(1) > P(1)))
                    && (P(0) < (polyVertices.getPoint(j)(0) - polyVertices.getPoint(i)(0))
                        * (P(1) - polyVertices.getPoint(i)(1))
                        / (polyVertices.getPoint(j)(1) - polyVertices.getPoint(i)(1))
                        + polyVertices.getPoint(i)(0)))
                c = !c;
        }
        return (bool)c;
    }

    void updateTMatrix();

    void updatePolyVertices();

public:
    bool hasModel() const
    {
        if (model_)
            return true;
        else
            return false;
    }

    void linkToStratigraphicModel(StratigraphicModelBasePtr mod);

    StratigraphicModelBasePtr getLinkedStratigraphicModel() const;

protected:
    ///
    /// \brief m_points the points defining a polyline in 3D
    spc::PolyLine3D verts_;

    /// things that will be auto-updated
    spc::PolyLine2D verts_2d_;

    /// plane on which to perform the projection
    PlanePtr proj_plane_;

    /// a distance limit from the projective plane
    float max_distance_;

    Transform<float, 3, Affine, AutoAlign> transform_;

    StratigraphicModelBasePtr model_;

private:
    friend class cereal::access;

    template <class Archive>
    void load(Archive& ar, std::uint32_t const version)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
            CEREAL_NVP(max_distance_),
            CEREAL_NVP(verts_),
            CEREAL_NVP(verts_2d_),
            CEREAL_NVP(proj_plane_));

        this->updateTMatrix();

        if (version >= 1)
            ar(CEREAL_NVP(model_));
    }

    template <class Archive>
    void save(Archive& ar, std::uint32_t const version) const
    {
        ar(cereal::base_class<spc::ElementBase>(this),
            CEREAL_NVP(max_distance_),
            CEREAL_NVP(verts_),
            CEREAL_NVP(verts_2d_),
            CEREAL_NVP(proj_plane_));

        if (version >= 1)
            ar(CEREAL_NVP(model_));
    }
};

} // end nspace

CEREAL_CLASS_VERSION(spc::SelectionRubberband, 1)
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(spc::SelectionRubberband, cereal::specialization::member_load_save)

#endif // SPCPLANARSELECTION_H
