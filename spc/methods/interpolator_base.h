#ifndef INTERPOLATOR_BASE_H
#define INTERPOLATOR_BASE_H


#include <spc/time_series/base_time_series.h>
#include <pcl/console/print.h>

#include <boost/foreach.hpp>
namespace spc
{

class InterpolatorBase: public spcElementBase
{
public:


    typedef boost::shared_ptr<InterpolatorBase> Ptr;
    typedef boost::shared_ptr<const InterpolatorBase> ConstPtr;

    InterpolatorBase();




    virtual float getInterpolatedValue(const float x_val) = 0;

    std::vector<float> getInterpolatedValues(const std::vector<float> new_x)
    {
        std::vector<float> out(new_x.size());


        int c = 0;
        BOOST_FOREACH(const float x, new_x)
        {
            out.at(c++) = getInterpolatedValue(x);
        }
        return out;
    }

    virtual void setInputSeries(GenericTimeSeries<float>::ConstPtr input)
    {
        if (!input)
        {
            pcl::console::print_error("[ERROR in %s] You are setting a null pointer", getClassName().c_str());
            return;
        }
        input_ = input;

        this->updateInternals();
    }

    virtual void getValidityRange(float &min, float &max)
    {

        if (!input_)
        {
            pcl::console::print_error("[ERROR in %s] You must define a valid input time series", getClassName().c_str());
            min = std::numeric_limits<float>::quiet_NaN ();
            max = std::numeric_limits<float>::quiet_NaN ();
            return;
        }

        min = input_->getMinX();
        max = input_->getMaxX();
    }

    // is automatically called after setting input
    virtual int updateInternals()
    {
        //nothing in this abstract class
    }


protected:
    GenericTimeSeries<float>::ConstPtr input_;


};



}//end nspace

#endif // INTERPOLATOR_BASE_H
