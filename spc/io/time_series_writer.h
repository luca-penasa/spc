#ifndef TIME_SERIES_WRITER_H
#define TIME_SERIES_WRITER_H

#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/elements/TimeSeriesSparse.h>

namespace spc
{

template <typename ScalarT>
class TimeSeriesWriter
{
public:
    TimeSeriesWriter();

    void setInputSeries(typename spc::GenericTimeSeries<ScalarT>::Ptr in_series)
    {

        in_series_ = in_series;

    }

    void setFilename(std::string filename) {filename_ = filename;}

    int writeAsciiAsSparse();

    void setASCIIPrecision(const int precision) {precision_ = precision;}

    void setSeparator(const std::string separator) {separator_ = separator;}

private:
    typename spc::GenericTimeSeries<ScalarT>::Ptr in_series_;
    std::string filename_;
    int precision_;
    std::string separator_;


};


} //end nspace
#endif // TIME_SERIES_WRITER_H
