#ifndef POINT3D_H
#define POINT3D_H

#include <spc/elements/ElementBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spc/elements/templated/PointSetBase.h>




namespace spc
{


// fwd delcarations
class PointCloudBase;


class SPC_LIB_API  Point3D : public ElementBase
{
public:
    SPC_ELEMENT(Point3D)
    EXPOSE_TYPE
    Point3D();

    Point3D(const float x, const float y, const float z);

    Point3D(const Eigen::Vector3f point);

    Point3D(const Point3D &other): ElementBase(other)
    {
        this->position_ = other.position_;
    }

    //! affine like accessors
    Eigen::Vector4f getPositionH() const;

    Eigen::Vector4f &getPositionH();

    //! clasical 3d accessors for retro-comatibility.
    //! notice you can write into the Point3D using what getPosition gives you back
    inline Eigen::Block<Eigen::Vector4f, 3, 1> getPosition()
    {
        return Eigen::Block<Eigen::Vector4f, 3, 1> (position_, 0, 0);
    }

    inline const Eigen::Block<const Eigen::Vector4f, 3, 1> getPosition() const
    {
        return Eigen::Block<const Eigen::Vector4f, 3, 1> (position_, 0, 0);
    }



    void getPosition(float &x, float &y, float &z) const;

    void setPosition(const Point3D &el);

    void setPosition(const Eigen::Vector3f position);

    void setPositionH(const Eigen::Vector4f position);

    //! this is deprecated. We are rying to move to the PoinCloudBase interface for cloud operations
//    void positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);

    void positionFromCentroid(const PointCloudXYZBase &cloud);

protected:
    Eigen::Vector4f position_; /**< we keep position in Homogeneous coords, it will be easier to apply transforms*/

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(position_));
    }
};

} // end nspace

#endif // END SAFEGFUARD
