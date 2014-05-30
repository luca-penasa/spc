#ifndef POINT3D_H
#define POINT3D_H

#include <spc/elements/ElementBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace spc
{
class MovableElement : public ElementBase
{
public:
    SPC_OBJECT(MovableElement)

    MovableElement();

    MovableElement(const float x, const float y, const float z);

    MovableElement(const Eigen::Vector3f point);

    Eigen::Vector3f getPosition() const;

    Eigen::Vector3f &getPosition();

    void getPosition(float &x, float &y, float &z) const;

    void setPosition(const MovableElement el);

    void setPosition(const Eigen::Vector3f position);

    void positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);

protected:
    Eigen::Vector3f position_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(position_));
    }
};

} // end nspace

#endif // END SAFEGFUARD
