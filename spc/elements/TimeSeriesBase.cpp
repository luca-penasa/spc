#include <spc/elements/TimeSeriesBase.h>
//#include <boost/random/random_device.hpp>

//#include <boost/random/uniform_real_distribution.hpp>
namespace spc
{


DtiClassType TimeSeriesBase::Type ("TimeSeriesBase", &ElementBase::Type);

void TimeSeriesBase::fill(const TimeSeriesBase::ScalarT value_)
{
    std::fill(this->y_.begin(), this->y_.end(), value_);
}

EigenTable::Ptr TimeSeriesBase::asEigenTable() const
{
    EigenTable::Ptr t(new EigenTable);

    std::vector<ScalarT> x = this->getX();
    std::vector<ScalarT> y = this->getY();

    t->resize(x.size());
    t->addNewComponent("x");
    t->addNewComponent("y");

    for (int i = 0; i < x.size(); ++i)
    {
        t->atScalar("x", i) = x.at(i);
        t->atScalar("y", i) = y.at(i);
    }

    return t;
}


}
