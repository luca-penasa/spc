#ifndef SPCPLANARSELECTION_H
#define SPCPLANARSELECTION_H

#include <spc/elements/ElementBase.h>
#include <spc/elements/PointCloudBase.h>
#include <spc/elements/Plane.h>
//#include <pcl/io/pcd_io.h> //for debug only
#include <spc/elements/PointCloudPcl.h>
#include <spc/elements/SelectionBase.h>
#include <boost/foreach.hpp>

#include <spc/elements/templated/PolyLine3D.h>

//#include <spc/elements/PointCloudSpc.h>

#include <spc/elements/StratigraphicModelBase.h>


#include <cereal/cereal.hpp>

#include <cereal/access.hpp>
namespace spc
{

/// NOTE this class must be splitted in a filter and a serializable object
/// it is not good that an object does operations on data. Filters do them.

class SelectionRubberband : public ElementBase, public SelectionOfPointsBase
{
public:
    SPC_ELEMENT(SelectionRubberband)
    EXPOSE_TYPE

    typedef Transform<float, 3, Affine, AutoAlign>  TransT;

    /** def const */
    SelectionRubberband();

    SelectionRubberband(const SelectionRubberband & other);

    SelectionRubberband(const PointCloudXYZBase &verts, float max_distance = 1);

    PolyLine3D getVertices() const
    {
        return verts_;
    }

    Plane getProjectionPlane() const
    {
        return proj_plane_;
    }

    void setVertices(const PointCloudXYZBase &verts)
    {
//        verts_ = PolyLine3D(verts);
        updateProjectionPlane();
        updatePolyVertices();
    }

    virtual bool contains(const Vector3f &obj) const override
    {

        if (proj_plane_.distanceTo(obj) > max_distance_)
            return false;
        else
        {
            Vector2f on_plane = (transform_ * obj).head(2);
            return isPointInPoly(on_plane, verts_2d_);
        }

    }

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
    bool isPointInPoly(const Eigen::Vector2f &P,
                      const PolyLine2D &polyVertices) const;

    void updateTMatrix()
    {
        transform_ = proj_plane_.get2DArbitraryRefSystem();
        DLOG(INFO) << "transform is "<< transform_.matrix();
    }


    void updatePolyVertices();

public:
    bool hasModel() const
    {
        if (model_)
            return true;
        else
            return false;
    }


    void linkToStratigraphicModel(spc::StratigraphicModelBase::Ptr mod)
    {
        if (hasModel())
        {
            LOG(WARNING) << "linked model changed";
        }
        model_ =  mod;
    }


    spc::StratigraphicModelBase::Ptr getLinkedStratigraphicModel() const
    {
        return model_;
    }

protected:

    ///
    /// \brief m_points the points defining a polyline in 3D
    spc::PolyLine3D verts_;

    /// things that will be auto-updated
    spc::PolyLine2D verts_2d_;

    /// plane on which to perform the projection
    Plane proj_plane_;

    /// a distance limit from the projective plane
    float max_distance_;


    Transform<float, 3, Affine, AutoAlign> transform_;


    spc::StratigraphicModelBase::Ptr model_;

private:
    friend class cereal::access;

	template <class Archive> void load(Archive &ar, std::uint32_t const version )
	{
		ar(cereal::base_class<spc::ElementBase>(this),
		   CEREAL_NVP(max_distance_),
		   CEREAL_NVP(verts_),
		   CEREAL_NVP(verts_2d_),
		   CEREAL_NVP(proj_plane_)
		);


		   this->updateTMatrix();

		if (version >= 1)
			ar(CEREAL_NVP(model_));
	}

	template <class Archive> void  save(Archive &ar, std::uint32_t const version) const
    {
        ar(cereal::base_class<spc::ElementBase>(this),
           CEREAL_NVP(max_distance_),
           CEREAL_NVP(verts_),
           CEREAL_NVP(verts_2d_),
           CEREAL_NVP(proj_plane_)
           );

		if (version >= 1)
            ar(CEREAL_NVP(model_));


    }



};




} // end nspace

//CEREAL_FORCE_DYNAMIC_INIT(spc)

CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES( spc::SelectionRubberband, cereal::specialization::member_load_save )


#endif // SPCPLANARSELECTION_H
