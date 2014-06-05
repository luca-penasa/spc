#ifndef ALIGNSINGLEATTITUDESTRATIGRAPHICMODELTOSAMPLES_H
#define ALIGNSINGLEATTITUDESTRATIGRAPHICMODELTOSAMPLES_H

#include <spc/elements/macros.h>
#include <vector>
namespace spc
{

// forward decl
spcFwdDeclSharedPtr(StratigraphicModelSingleAttitude)
spcFwdDeclSharedPtr(SamplesDB)

class AlignSingleAttitudeStratigraphicModelToSamples
{
public:
    AlignSingleAttitudeStratigraphicModelToSamples();

    void setFieldName(const std::string &name)
    {
        strat_pos_field_ = name;
    }

    void setStraigraphicModel(const StratigraphicModelSingleAttitudePtr model)
    {
        model_ = model;
    }

    void setSamplesDB(const SamplesDBPtr db)
    {
        db_ = db;
    }
    int compute();

    std::vector<float> getResiduals() const
    {
        return residuals_;
    }


private:
    void updateResiduals();

    void updateShift();

protected:
    //! the model to modify
    StratigraphicModelSingleAttitudePtr model_;

    //! the list of samples to be used
    SamplesDBPtr db_;

    //! the field to use as stratigraphic position of samples.
    std::string strat_pos_field_;

private:
    std::vector<float> model_sp_;
    std::vector<float> real_sp_;
    std::vector<float> residuals_;

    float shift_;
};

} // end nspace

#endif // ALIGNSINGLEATTITUDESTRATIGRAPHICMODELTOSAMPLES_H
