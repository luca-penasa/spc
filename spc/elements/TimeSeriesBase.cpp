#include <spc/elements/TimeSeriesBase.h>
//#include <boost/random/random_device.hpp>

//#include <boost/random/uniform_real_distribution.hpp>
namespace spc
{


DtiClassType TimeSeriesBase::Type ("TimeSeriesBase", &ElementBase::Type);

void TimeSeriesBase::fill(const TimeSeriesBase::ScalarT value_)
{
    y_.fill(value_);
}

EigenTable::Ptr TimeSeriesBase::asEigenTable() const
{
    EigenTable::Ptr t(new EigenTable);

    VectorT x = this->getX();
    VectorT y = this->getY();

    t->resize(x.size());
    t->addNewComponent("x");
    t->addNewComponent("y");

    t->column("x") = x;
    t->column("y") = y;

    return t;
}


}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::TimeSeriesBase)
