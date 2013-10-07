#ifndef CC_GEOLOGIC_PLANE_H
#define CC_GEOLOGIC_PLANE_H

#include "ccArrow.h"
#include "ccPlane.h"
#include <spc/elements/orientation.h>

#include "DistanceComputationTools.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"


///
/// \brief The ccPlaneOrientation class gives a qCC-valid representation of an orientation in space
///
class ccOrientation: public ccGenericPrimitive, public spc::Orientation

{
public:
    ccOrientation(CCVector3 center,
                       CCVector3 orientation,
                       float scale):
        Orientation(asEigenVector(center), asEigenVector(orientation))
    {
        m_center  = center;
        m_orientation = orientation;
        m_scale = scale;


        buildUp();
    }


    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_ARROW;}


    //! inherited from ccGenericPrimitive
    virtual QString getTypeName() const { return "GeoOrientation"; }


    //    virtual bool hasDrawingPrecision() const { return true; }


    virtual ccGenericPrimitive* clone() const
    {
        return finishCloneJob(new ccOrientation(m_center, m_orientation, m_scale));
    }


    virtual bool buildUp()
    {
        CCVector3  newcenter;
        ccPlane * plane;

            //we add also a plane representing the local plane
            plane = new ccPlane(3*m_scale, 3*m_scale, get_orientating_matrix(m_center, m_orientation));
            *this += ccArrow(m_scale, get_orientating_matrix(m_center, m_orientation), QString("Plane Orientation"));



        this->addChild(plane);
    }


    static ccPlane * fromPlaneAndCloud(const CCVector3 c,
                                       const CCVector3 n,
                                       CCLib::GenericIndexedCloudPersist *cloud,
                                       CCVector3 &new_c)
{

    size_t count = cloud->size();

    CCLib::Neighbourhood Yk(cloud);

    //plane equation
    PointCoordinateType theLSQPlane[4] = {0,0,0,0}; /* = Yk.getLSQPlane();*/

    theLSQPlane[0] = n.x;
    theLSQPlane[1] = n.y;
    theLSQPlane[2] = n.z;
    theLSQPlane[3] = c.dot(n);



    if (!theLSQPlane | (count < 3))
    {
        ccLog::Warning("[ccGenericPointCloud::fitPlane] Not enough points to fit a plane!");
        return 0;
    }

    //get the centroid - we use the position as centroid
    const CCVector3 * G = &c;

    //and a local base
    CCVector3 N(theLSQPlane);
    const CCVector3* X = Yk.getLSQPlaneX(); //main direction
    assert(X);
    CCVector3 Y = N * (*X); //second direction on the local RS - cross prod

    PointCoordinateType minX=0,maxX=0,minY=0,maxY=0;
    cloud->placeIteratorAtBegining();

    for (unsigned k=0;k<count;++k)
    {
        //projetion into local 2D plane ref.
        CCVector3 P = *(cloud->getNextPoint()) - *G;
        PointCoordinateType x2D = P.dot(*X);
        PointCoordinateType y2D = P.dot(Y);

        if (k!=0)
        {
            if (minX<x2D)
                minX=x2D;
            else if (maxX>x2D)
                maxX=x2D;
            if (minY<y2D)
                minY=y2D;
            else if (maxY>y2D)
                maxY=y2D;
        }
        else
        {
            minX=maxX=x2D;
            minY=maxY=y2D;
        }
    }


    //we recenter plane (as it is not always the case!)
    float dX = maxX-minX;
    float dY = maxY-minY;
    CCVector3 Gt = *G + *X * (minX+dX*0.5);
    Gt += Y * (minY+dY*0.5);
    ccGLMatrix glMat(*X,Y,N,Gt);

    ccPlane * plane = new ccPlane(dX, dY, &glMat);

    return plane;
}




private:

CCVector3 m_center;
CCVector3 m_orientation;
float m_scale;



static ccGLMatrix * get_orientating_matrix(CCVector3 center, CCVector3 orientation)
{
    ccGLMatrix * mat = new ccGLMatrix;
    *mat = ccGLMatrix::FromToRotation(orientation, CCVector3(0,0,1));
    mat->setTranslation(center);

    return mat;
}

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
