#ifndef TIMESERIESBANDPASSFILTER_H
#define TIMESERIESBANDPASSFILTER_H

#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <algorithm>

#include <spc/methods/GenericFilter.h>
namespace spc
{


class TimeSeriesBandPassFilter: public GenericFilter
{
public:

    spcTypedefSharedPtrs(TimeSeriesBandPassFilter)

    enum BACKEND { EIGENFFT};
    enum NAN_MANAGEMENT {AVERAGE};

    TimeSeriesBandPassFilter();

    void setTimeSeries(TimeSeriesEquallySpaced::Ptr series)
    {
        series_ = series;
    }


    /** returns a new ts filtered
     */
    spc::TimeSeriesEquallySpaced::Ptr filter_aquila();
    spc::TimeSeriesEquallySpaced::Ptr filter_eigen();


    spcSetGetMacro(LowFrequency, f_low_, float)

    spcSetGetMacro(HighFrequency, f_high_, float)

    spcSetGetMacro(PreserveMean, preserve_mean_, bool)



protected:
    TimeSeriesEquallySpaced::Ptr series_;
    TimeSeriesEquallySpaced::Ptr out_series_;




    float f_low_ = 1;
    float f_high_ = 2;

    BACKEND backend = EIGENFFT;

    bool preserve_mean_ = true;


    // GenericFilter interface
public:
    virtual InputRequirements getInputRequirements() const override;


    virtual void doComputations() override;



    };






}// end nspace
#endif // TIMESERIESBANDPASSFILTER_H
