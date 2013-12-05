#ifndef CONTINOUS_VALUES_STRATIGRAPHY_H
#define CONTINOUS_VALUES_STRATIGRAPHY_H

#include "element_base.h"
#include <spc/time_series/equally_spaced_time_series.h>

namespace spc
{
class ContinousValuesLog: public spcElementBase

{
public:
    ContinousValuesLog();

    float getStratigraphicStart() const {return stratigraphic_start_;}

    void setMinStratigraphicPosition(float sp_start) {stratigraphic_start_ = sp_start;}

    float getStratigraphicPositionOfSample(int sample_id) const
    {
        return sample_id * sampling_step_ + stratigraphic_start_;
    }

    float getMaxStratigraphicPosition() const
    {
        return getStratigraphicPositionOfSample(values_.size());
    }

    void resize(int number)
    {
        values_.resize(number);
    }

    size_t getSize() const
    {
        return values_.size();
    }

    void setValue(int id, float value)
    {
        assert (id < values_.size());
        values_.at(id) = value;
    }

    float getValue(int id) const
    {
        assert (id < getSize());
        return values_.at(id);
    }

    template <typename OutT>
    std::vector<OutT> getStratigraphicPositions() const
    {
        std::vector<OutT> sps;
        sps.resize(getSize());
        for (int i = 0; i < getSize(); ++i)
        {
            sps.at(i) = (OutT) getStratigraphicPositionOfSample(i);
        }

        return sps;
    }



    template <typename OutT>
    std::vector<OutT> getValues() const
    {
        std::vector<OutT> val;
        val.resize(getSize());


        for (int i  = 0 ; i < getSize(); ++i)
            val.at(i) = (OutT) getValue(i);

        return val;
    }

    std::vector<float> getValues() const
    {
        return values_;
    }




    float getSamplingStep() const {return sampling_step_;}

    void setSamplingStep(float s_step) {sampling_step_ = s_step;}


protected:
    float sampling_step_;
    std::vector<float> values_;
    float stratigraphic_start_;
};

}//end nspace

#endif // CONTINOUS_VALUES_STRATIGRAPHY_H
