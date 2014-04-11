#ifndef NORMAL3D_H
#define NORMAL3D_H
#include <spc/elements/element_base.h>
#include <pcl/features/normal_3d.h>


namespace spc
{


class spcNormal3D: public spcElementBase
{
public:

    typedef boost::shared_ptr<spcNormal3D> Ptr;
    typedef boost::shared_ptr<const spcNormal3D> ConstPtr;


    spcNormal3D();

    spcNormal3D(float x, float y, float z)
    {
        normal_ = Vector3f(x,y,z);
    }

    spcNormal3D(const Vector3f v)
    {
        normal_ = v;
    }

    /// this project a given 3d point onto the normal
    Vector3f projectPoint(const Vector3f &point) const
    {
        return (this->getUnitNormal() * this->getUnitNormal().dot(point));
    }

    void setNormal(const Vector3f n)
    {
        normal_ = n;
    }

    void setNormal(const spcNormal3D n)
    {
        *this = n;
    }



    Vector3f getNormal() const
    {
        return normal_;
    }

    void normalize()
    {
        normal_.normalize();
    }

    Vector3f getUnitNormal() const
    {
        Vector3f unit = normal_;
        unit.normalize();
        return unit;
    }

    /// invert the direction of the normal.
    void flipNormal()
    {
        normal_ = - normal_;
    }

    void setUnitAxis(const int ax_id = 2)
    {
        normal_.setZero();
        normal_(ax_id) = 1.0;
    }

    void normalFromBestFit(pcl::PointCloud<pcl::PointXYZ> & cloud);

protected:



    Vector3f normal_;

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
//        ar & boost::serialization::make_nvp("normal", normal_);
        ar & BOOST_SERIALIZATION_NVP(normal_);
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(spcElementBase);
    }


};


} //end nspace

#endif // NORMAL3D_H
