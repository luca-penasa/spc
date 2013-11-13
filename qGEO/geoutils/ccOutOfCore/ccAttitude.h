#ifndef CC_GEOLOGIC_PLANE_H
#define CC_GEOLOGIC_PLANE_H


#include <spc/elements/attitude.h>

//#include "DistanceComputationTools.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"





///
/// \brief The ccAttitude class gives a qCC-valid representation of a geological attitude
///
class ccAttitude: public ccHObject, public spc::Attitude

{
public:
    ccAttitude(CCVector3 center, CCVector3 orientation);

    ccAttitude(spc::Attitude att);

    ccAttitude();

    //inherited methods (ccHObject)
    virtual bool isSerializable() const { return true; }
    virtual bool hasColors() const { return true; }
    virtual ccBBox getMyOwnBB();



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


};//end class

#endif // CCPLANEORIENTATION_H
