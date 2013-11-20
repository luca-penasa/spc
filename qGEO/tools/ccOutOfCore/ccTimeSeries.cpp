#include "ccTimeSeries.h"

ccTimeSeries::ccTimeSeries()
{
    initMetaData();
}

ccTimeSeries::ccTimeSeries(const ccTimeSeries &other): spc::EquallySpacedTimeSeries<float>(other)
{
    initMetaData();
}

ccTimeSeries::ccTimeSeries(const spc::EquallySpacedTimeSeries<float> &other) : spc::EquallySpacedTimeSeries<float>(other)
{
    initMetaData();
}

void ccTimeSeries::initMetaData()
{
    QVariant var(QString("Is a time Series"));
    setMetaData(QString("[qGEO][ccTimeSeries]"), var);
}
