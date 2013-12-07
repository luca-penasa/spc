#ifndef CC_GEOLOGIC_PLANE_H
#define CC_GEOLOGIC_PLANE_H


#include <spc/elements/attitude.h>

//#include "DistanceComputationTools.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"


#include <ccOutOfCore/ccMyBaseObject.h>


#include <QIcon>




///
/// \brief The ccAttitude class gives a qCC-valid representation of a geological attitude
///
class ccAttitude:public spc::spcAttitude,  public ccMyBaseObject

{
public:
    ccAttitude(CCVector3 center, CCVector3 orientation);

    ccAttitude(spc::spcAttitude att);

    ccAttitude();

    //inherited methods (ccHObject)
    virtual bool isSerializable() const { return true; }
    virtual bool hasColors() const { return true; }
    virtual ccBBox getMyOwnBB();

    virtual QIcon * getIcon() const
    {
        return new QIcon(QString::fromUtf8(":/toolbar/icons/attitude.png"));
    }




protected:

    //    void setAttitudeAsMetadata();

    virtual void drawMeOnly(CC_DRAW_CONTEXT &context);
    virtual void applyGLTransformation(const ccGLMatrix& trans) ;
    virtual void setGLTransformation(const ccGLMatrix& trans);


    void initParameters();
    void initMetadata();

    float m_scale;
    float m_scale_factor;

    int m_width;

    //    ccGLMatrix m_oldTransform;


    static Eigen::Vector3f asEigenVector(CCVector3 v)
    {
        return Eigen::Vector3f (v.x, v.y, v.z); //we should make a MAP instead than a copy!
    }

    static CCVector3 asCCVector(Eigen::Vector3f v)
    {
        return CCVector3(v(0),v(1), v(2));
    }


    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(m_scale);
        ar & BOOST_SERIALIZATION_NVP(m_width);
        ar & boost::serialization::make_nvp("spcAttitude", boost::serialization::base_object<spc::spcAttitude> (*this));
        ar & boost::serialization::make_nvp("ccMyBaseObject", boost::serialization::base_object<ccMyBaseObject> (*this));

    }



};//end class

#endif // CCPLANEORIENTATION_H
