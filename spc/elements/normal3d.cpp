#include "normal3d.h"
namespace spc
{


spcNormal3D::spcNormal3D() : normal_(0,0,1)
{
}

void spcNormal3D::normalFromBestFit(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    float curv;
    Vector4f params;

    //do the fit
    pcl::computePointNormal(cloud, params, curv);

    Vector3f n = params.head(3);
    n.normalize();
    setNormal(n);
}



}//end nspace


BOOST_CLASS_EXPORT_GUID(spc::spcNormal3D, "spcNormal3D")
