#ifndef CCPLANEORIENTATION_H
#define CCPLANEORIENTATION_H

#include "ccArrow.h"
#include "ccPlane.h"

class ccPlaneOrientation: public ccArrow

{
public:
    ccPlaneOrientation(CCVector3 center, CCVector3 orientation, float scale):
        ccArrow(scale, get_orientating_matrix(center, orientation), QString("Plane Orientation")),
        m_center(center),
        m_orientation(orientation)

    {
        //we add also a plane representing the local plane
       *this += ccPlane(3*m_scale, 3*m_scale, get_orientating_matrix(center, orientation));
    }


private:
    CCVector3 m_orientation;
    CCVector3 m_center;

    static ccGLMatrix * get_orientating_matrix(CCVector3 center, CCVector3 orientation)
    {
        ccGLMatrix * mat = new ccGLMatrix;
        *mat = ccGLMatrix::FromToRotation(orientation, CCVector3(0,0,1));
        mat->setTranslation(center);

        return mat;
    }

};

#endif // CCPLANEORIENTATION_H
