#ifndef SPCPLANARSELECTION_H
#define SPCPLANARSELECTION_H

#include <spc/elements/ElementBase.h>
#include <spc/elements/PointCloudBase.h>
#include <spc/elements/Plane.h>
#include <pcl/io/pcd_io.h> //for debug only
#include <spc/elements/PointCloudPcl.h>
//#include <boost/serialization/shared_ptr.hpp>

#include <spc/elements/SelectionBase.h>
#include <boost/foreach.hpp>

#include <spc/elements/templated/PolyLine3D.h>

namespace spc
{

/// NOTE this class must be splitted in a filter and a serializable object
/// it is not good that an object does operations on data. Filters do them.

class SelectionRubberband : public ElementBase, public SelectionOfPointsBase
{
public:
    SPC_OBJECT(SelectionRubberband)
    EXPOSE_TYPE
    typedef pcl::PointCloud<pcl::PointXYZ> cloudT;


    /** def const */
    SelectionRubberband()
    {

    }

    SelectionRubberband(const PointCloudXYZBase &verts, float max_distance = 1): max_distance_(max_distance)
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

//    SelectionRubberband(const SelectionRubberband &el)
//    {
//        verts_ = el.getVertices();
//        updateProjectionPlane();
//        updatePolyVertices();
//    }

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

    virtual bool isInsideSelection(const Vector3f &obj) const
    {
        Vector2f on_plane = proj_plane_.projectOnPlane(obj).head(2);
        DLOG(INFO) << "to be implemented";
    }

    float getMaxDistance() const
    {
        return max_distance_;
    }

    void setMaxDistance(float d)
    {
        max_distance_ = d;
    }

//    void setInputCloud(PointCloudBase::Ptr cloud)
//    {
//        in_cloud_ = cloud;
//        updateProjectedCloud();
//        updateIndices();
//    }

//    std::vector<int> getIndices()
//    {
//        return indices_;
//    }

    void updateIndices()
    {
//        DLOG(INFO) << "Updating indices";
//        for (int i = 0; i < projected_cloud_.size(); ++i)
//        {
//            if (projected_cloud_.at(i).z <= max_distance_)
//            {
//                pcl::PointXY p;
//                p.x = projected_cloud_.at(i).x;
//                p.y = projected_cloud_.at(i).y;
//                if (isPointInPoly(p, verts_2d_))
//                    indices_.push_back(i);
//            }
//        }
//        DLOG(INFO) << "Updating indices. Done";
    }

//    pcl::PointCloud<pcl::PointXYZ> getInside(PointCloudBase::Ptr in_cloud)
//    {
//        setInputCloud(in_cloud);
//        updateProjectedCloud();
//        updatePolyVertices();
//        updateIndices();

//        std::vector<int> indices = getIndices();

//        std::cout << "found " << indices.size() << " valid indices\n"
//                  << std::endl;

//        // now filter out

//        pcl::PointCloud<pcl::PointXYZ> cloud;
//        cloud.resize(indices.size());

//        for(auto id: indices)
//        {
//            Vector3f point = in_cloud->getPoint(id);
//            pcl::PointXYZ p;
//            p.x = point(0);
//            p.y = point(1);
//            p.z = point(2);
//            cloud.push_back(p);
//        }

//        return cloud;
//    }

protected:
//    void updateProjectedCloud()
//    {
//        DLOG(INFO) <<"Updating projected cloud.";
//        projected_cloud_ = in_cloud_->applyTransform(proj_plane_.get2DArbitraryRefSystem());

//        DLOG(INFO) << "Projected cloud has N points" << projected_cloud_.size();
//        pcl::io::savePCDFileBinary("/home/luca/tmp.pcd", projected_cloud_);
//        DLOG(INFO) <<"Updating projected cloud. Done";


//    }

    void updateProjectionPlane()
    {
        DLOG(INFO) << "Updating projection plane.";
        proj_plane_.positionFromCentroid(verts_);

        DLOG(INFO) << "Centroid is " << proj_plane_.getPosition().transpose();
        Normal3D n;
        n.normalFromBestFit(verts_);
        proj_plane_.setNormal(n.getNormal());
        LOG(INFO) << "Prjection plane normal " <<  n.getNormal().transpose();
    }

    /// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
    inline int isPointInPoly(const pcl::PointXY P,
                      const std::vector<pcl::PointXY> &polyVertices)
    {
        int nvert = polyVertices.size();
        int i, j, c = 0;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((polyVertices[i].y > P.y) != (polyVertices[j].y > P.y))
                && (P.x < (polyVertices[j].x - polyVertices[i].x)
                          * (P.y - polyVertices[i].y)
                          / (polyVertices[j].y - polyVertices[i].y)
                          + polyVertices[i].x))
                c = !c;
        }
        return c;
    }


    void updatePolyVertices()
    {

       DLOG(INFO) << "Updating poly vertices";

        DLOG(INFO) << "N initial poly verts: " << verts_.getNumberOfPoints();


        PolyLine3D proj = verts_.transform<PolyLine3D>(proj_plane_.get2DArbitraryRefSystem());


        DLOG(INFO) << "Cloud projected! copying as 2d cloud";
        verts_2d_.resize(proj.getNumberOfPoints());
        for(int i = 0; i < proj.getNumberOfPoints(); ++i)
        {
            verts_2d_.setPoint(i, proj.getPoint(i).head(2));
        }

        DLOG(INFO) << "there are "<< verts_2d_.getNumberOfPoints() <<  " poly verts";
    }

    ///
    /// \brief m_points the points defining a polyline in 3D
    ///
    spc::PolyLine3D verts_;


    /// things that will be auto-updated
    spc::PolyLine2D verts_2d_;

    /// plane on which to perform the projection
    Plane proj_plane_;

    /// a distance limit from the projective plane
    float max_distance_;


//    /// this stuff will go into the evaluator.
//    ///
//    /// \brief in_cloud_ it the input cloud to be segmented
//    ///
//    PointCloudBase::Ptr in_cloud_;

//    /// good indexes (inside)
//    std::vector<int> indices_;

//    /// the input cloud projected in the 2d ref system
//    /// z is the distance from the proj_plane_
//    pcl::PointCloud<pcl::PointXYZ> projected_cloud_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
           CEREAL_NVP(max_distance_),
           CEREAL_NVP(verts_),
           CEREAL_NVP(verts_2d_),
           CEREAL_NVP(proj_plane_));
    }

    // ElementBase interface

    // SelectionBase interface
};

} // end nspace

#endif // SPCPLANARSELECTION_H
