#ifndef CC_GEOLOGIC_PLANE_H
#define CC_GEOLOGIC_PLANE_H

#include "ccArrow.h"
#include "ccPlane.h"
#include <spc/elements/attitude.h>

#include "DistanceComputationTools.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

//#include "ccMyCCHObject.h"


///
/// \brief The ccAttitude class gives a qCC-valid representation of a geological attitude
///
class ccAttitude: public ccHObject, public spc::Attitude

{
public:
    ccAttitude(CCVector3 center, CCVector3 orientation);



    //! inherited from ccGenericPrimitive
    virtual QString getTypeName() const { return "Attitude"; }


//    virtual ccGenericPrimitive* clone() const
//    {
//        return finishCloneJob(new ccAttitude(m_center, m_orientation, m_scale));
//    }


//    virtual bool buildUp();


//    static ccPlane * fromPlaneAndCloud(const CCVector3 c,
//                                       const CCVector3 n,
//                                       CCLib::GenericIndexedCloudPersist *cloud,
//                                       CCVector3 &new_c);


    virtual void drawMeOnly(CC_DRAW_CONTEXT &context)
    {
        glPushAttrib(GL_LINE_BIT);
        glLineWidth(1);

        //we draw the segments
        if (isSelected())
            glColor3ubv(ccColor::red);
        else
            glColor3ubv(ccColor::green);
        glBegin(GL_LINES);

        glVertex3fv(position_.data());

        glEnd();
        glPopAttrib();
    }


private:

//    CCVector3 m_center;
//    CCVector3 m_orientation;
//    float m_scale;



//    static ccGLMatrix * get_orientating_matrix(CCVector3 center, CCVector3 orientation);

    static Eigen::Vector3f asEigenVector(CCVector3 v)
    {
        return  Eigen::Vector3f (v.x, v.y, v.z); //we should make a MAP instead than a copy!
    }

    static CCVector3 asCCVector(Eigen::Vector3f v)
    {
        return CCVector3(v(0),v(1), v(2));
    }


};//end class

#endif // CCPLANEORIENTATION_H
