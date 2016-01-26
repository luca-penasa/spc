#include <spc/elements/TimeSeriesBase.h>

namespace spc
{


DtiClassType TimeSeriesBase::Type ("TimeSeriesBase", &ElementBase::Type);

void TimeSeriesBase::fill(const TimeSeriesBase::ScalarT value_)
{
    y_.fill(value_);
}

int TimeSeriesBase::toAsciiStream(std::ostream &stream) const
{
    stream << "x y" << std::endl;

    Eigen::Matrix<float, -1, 2> mat;

    mat.resize(this->getNumberOfSamples(), Eigen::NoChange);
    mat.col(0) << this->getX();
    mat.col(1) << this->getY();

    stream << mat;

}


}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::TimeSeriesBase)
