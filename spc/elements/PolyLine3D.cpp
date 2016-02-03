#include "PolyLine3D.h"
namespace spc {

DtiClassType PolyLine3D::Type("PolyLine3D", &GeometricElement3DBase::Type);

void PolyLine3D::applyTransform(const GeometricElement3DBase::TransformT &transform)
{
    for (int i = 0; i < this->getNumberOfPoints(); ++i)
    {
        Eigen::Vector3f p = data_.row(i);
        data_.row(i) = transform * p;
    }
}

}
