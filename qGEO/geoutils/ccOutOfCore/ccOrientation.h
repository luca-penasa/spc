#ifndef CC_GEOLOGIC_PLANE_H
#define CC_GEOLOGIC_PLANE_H

#include "ccArrow.h"
#include "ccPlane.h"
#include <spc/elements/orientation.h>

#include "DistanceComputationTools.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

#include "ccMyCCHObject.h"


///
/// \brief The ccPlaneOrientation class gives a qCC-valid representation of an orientation in space
///
class ccOrientation: public ccGenericPrimitive, public spc::Orientation, public ccMyHObject

{
public:
    ccOrientation(CCVector3 center,
                  CCVector3 orientation,
                  float scale);


    virtual void populateTreeView(ccProperties *prop_widget) {}

    //! Returns class ID


    virtual CC_CLASS_ENUM getClassID() const { return static_cast<CC_CLASS_ENUM> (MY_CC_ORIENTATION); }

    //! inherited from ccGenericPrimitive
    virtual QString getTypeName() const { return "GeoOrientation"; }


    //    virtual bool hasDrawingPrecision() const { return true; }


    virtual ccGenericPrimitive* clone() const
    {
        return finishCloneJob(new ccOrientation(m_center, m_orientation, m_scale));
    }


    virtual bool buildUp();


    static ccPlane * fromPlaneAndCloud(const CCVector3 c,
                                       const CCVector3 n,
                                       CCLib::GenericIndexedCloudPersist *cloud,
                                       CCVector3 &new_c);




private:

    CCVector3 m_center;
    CCVector3 m_orientation;
    float m_scale;



    static ccGLMatrix * get_orientating_matrix(CCVector3 center, CCVector3 orientation);

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
