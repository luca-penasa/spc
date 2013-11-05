#include "ccOrientation.h"
#include <iostream>



ccOrientation::ccOrientation(CCVector3 center, CCVector3 orientation, float scale):
    Orientation(asEigenVector(center), asEigenVector(orientation))
{
    m_center  = center;
    m_orientation = orientation;
    m_scale = scale;


    buildUp();
}

bool ccOrientation::buildUp()
{
    CCVector3  newcenter;
    ccPlane * plane;

    //we add also a plane representing the local plane
    plane = new ccPlane(3*m_scale, 3*m_scale, get_orientating_matrix(m_center, m_orientation));
    *this += ccArrow(m_scale, get_orientating_matrix(m_center, m_orientation), QString("Plane Orientation"));

    this->addChild(plane);


}

ccPlane *ccOrientation::fromPlaneAndCloud(const CCVector3 c, const CCVector3 n, CCLib::GenericIndexedCloudPersist *cloud, CCVector3 &new_c)
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

ccGLMatrix *ccOrientation::get_orientating_matrix(CCVector3 center, CCVector3 orientation)
{
    ccGLMatrix * mat = new ccGLMatrix;
    *mat = ccGLMatrix::FromToRotation(orientation, CCVector3(0,0,1));
    mat->setTranslation(center);

    return mat;
}
