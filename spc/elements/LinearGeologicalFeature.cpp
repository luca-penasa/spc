#include "LinearGeologicalFeature.h"
namespace spc
{

DtiClassType LinearGeologicalFeature::Type("LinearGeologicalFeature", &StratigraphicPositionableElement::Type);


LinearGeologicalFeature::LinearGeologicalFeature()
{

}

float LinearGeologicalFeature::predictStratigraphicPositionFromModel() const
{
    // a linear features lies at the average of its points strat. pos.

    if (this->getStratigraphicModel() == nullptr)
            return spcNANMacro;

    float av = 0;

    for (int i =0; i < polyline_.getNumberOfPoints(); ++i)
    {
        av += this->getStratigraphicModel()->predictStratigraphicPosition(polyline_.getPoint(i));
    }

    av /= polyline_.getNumberOfPoints();


    return av;

}

void LinearGeologicalFeature::applyTransform(const GeometricElement3DBase::TransformT &transform)
{
    polyline_.applyTransform(transform);
}



}//end nspace



#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::LinearGeologicalFeature)
