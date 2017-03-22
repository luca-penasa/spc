#pragma once
#ifndef SPCPLANARSELECTION_H
#define SPCPLANARSELECTION_H

#include <spc/elements/GeometricElement3DBase.h>
#include <spc/elements/SelectionBase.h>
#include <spc/elements/PolyLine3D.h>

namespace spc {
spcFwdDeclSharedPtr(StratigraphicModelBase)
spcFwdDeclSharedPtr(Plane)

/// NOTE this class must be splitted in a filter and a serializable object
/// it is not good that an object does operations on data. Filters do them.

class SelectionRubberband : public GeometricElement3DBase, public SelectionOfPointsBase {
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


    static inline bool isPointInPoly(const Eigen::Vector2f& P,
                       const PolyLine2D& polyVertices)
    {
        int nvert = polyVertices.getNumberOfPoints();
        int i, j = 0;
        bool inside = false;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((polyVertices.getPoint(i)(1) > P(1)) != (polyVertices.getPoint(j)(1) > P(1)))
                    && (P(0) < (polyVertices.getPoint(j)(0) - polyVertices.getPoint(i)(0))
                        * (P(1) - polyVertices.getPoint(i)(1))
                        / (polyVertices.getPoint(j)(1) - polyVertices.getPoint(i)(1))
                        + polyVertices.getPoint(i)(0)))
                inside = !inside;
        }
        return inside;
    }

    //! from http://alienryderflex.com/polygon/ - we can implement their faster version with precompued vars
    static inline bool isPointInPoly2(const Eigen::Vector2f& P,
                                      const PolyLine2D& polyVertices) {

        int   j=polyVertices.getNumberOfPoints()-1 ;
        bool  oddNodes=false      ;

        for (int i=0; i<polyVertices.getNumberOfPoints(); i++)
        {
            if ((polyVertices.getPoint(i)(1)< P(1) && polyVertices.getPoint(j)(1)>=P(1)
                 ||   polyVertices.getPoint(j)(1)< P(1) && polyVertices.getPoint(i)(1)>=P(1))
                    &&  (polyVertices.getPoint(i)(0)<=P(0) || polyVertices.getPoint(j)(0)<=P(0)))
            {
                oddNodes^=(polyVertices.getPoint(i)(0) + (P(1) - polyVertices.getPoint(i)(1)) /
                           (polyVertices.getPoint(j)(1) - polyVertices.getPoint(i)(1)) *
                           (polyVertices.getPoint(j)(0) - polyVertices.getPoint(i)(0)) < P(0));
            }
            j=i;
        }

        return oddNodes;
    }


protected:
    void updateProjectionPlane();

    /// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
    /// int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
//{
//  int i, j, c = 0;
//  for (i = 0, j = nvert-1; i < nvert; j = i++) {
//    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
//     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
//       c = !c;
//  }
//  return c;
//}


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
        ar(cereal::base_class<spc::GeometricElement3DBase>(this),
            CEREAL_NVP(max_distance_),
            CEREAL_NVP(verts_),
            CEREAL_NVP(verts_2d_),
            CEREAL_NVP(proj_plane_));

        this->updateTMatrix();
        this->updatePolyVertices();
        this->updateProjectionPlane();


        if (version >= 1)
            ar(CEREAL_NVP(model_));
    }

    template <class Archive>
    void save(Archive& ar, std::uint32_t const version) const
    {
        ar(cereal::base_class<spc::GeometricElement3DBase>(this),
            CEREAL_NVP(max_distance_),
            CEREAL_NVP(verts_),
            CEREAL_NVP(verts_2d_),
            CEREAL_NVP(proj_plane_));

        if (version >= 1)
            ar(CEREAL_NVP(model_));
    }

    // GeometricElement3DBase interface
    public:
    virtual void applyTransform(const TransformT &transform) override;
};

} // end nspace

CEREAL_CLASS_VERSION(spc::SelectionRubberband, 1)
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(spc::SelectionRubberband, cereal::specialization::member_load_save)

#endif // SPCPLANARSELECTION_H
