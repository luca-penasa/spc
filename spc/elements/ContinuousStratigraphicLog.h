#ifndef CONTINUOUSSTRATIGRAPHICLOG_H
#define CONTINUOUSSTRATIGRAPHICLOG_H

#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
namespace spc
{


class ContinuousStratigraphicLog: public StratigraphicPositionableElement
{

public:
    SPC_ELEMENT(ContinuousStratigraphicLog)
    EXPOSE_TYPE


    ContinuousStratigraphicLog();

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<StratigraphicPositionableElement>(this));
        ar(ts_);

    }

    virtual float predictStratigraphicPositionFromModel() const override
    {
        spc::StratigraphicModelBase::Ptr model = this->getStratigraphicModel();
        if (model != nullptr)
            return model->getStratigraphicShift();
        else
            return spcNANMacro; // returning nan
    }




protected:
    TimeSeriesEquallySpaced ts_;



    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const TransformT &transform) override
    {
        // nothing to do for the time series.
        // \todo solve this wrong case.
    }



    // StratigraphicPositionableElement interface
public:
    virtual void setUserStratigraphicPosition(const float &_arg) override;
    virtual float getUserStratigraphicPosition() const override;
};




}// end nspace

#endif // CONTINUOUSSTRATIGRAPHICLOG_H
