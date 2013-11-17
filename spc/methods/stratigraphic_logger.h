#ifndef STRATIGRAPHIC_LOGGER_H
#define STRATIGRAPHIC_LOGGER_H

#include <spc/elements/generic_cloud.h>
#include <spc/stratigraphy/stratigraphic_model_base.h>
#include <spc/methods/time_series_generator.h>


namespace spc
{

class StratigraphicLogger
{
public:
    StratigraphicLogger();


    void setInputCloud(GenericCloud * cloud)
    {
        cloud_ = cloud;

    }

    void setStratigraphicModel(StratigraphicModelBase * model)
    {
        model_ = model;
    }

    void compute()
    {
        fillIndicesIfNeeded();
        stratigraphic_positions_ = model_->getStratigraphicPositions(cloud_, indices_);

        //and the scalar field
        std::vector<float> field = cloud_->getField(fieldname, indices_);

        // and now compute the time series
        TimeSeriesGenerator generator;
        generator.setIndices(indices_);
        generator.setInputReader();

    }


    void setStep(const float step)
    {
        step_ = step;
    }

    void setBandWidth(const float band)
    {
        bandwidth_ = band;
    }

    void setIndices(const std::vector<int> indices)
    {
        indices_ = indices;
    }


    void setFieldName(std::string fieldname)
    {
        fieldname_ = fieldname;
    }





protected:

    void fillIndicesIfNeeded()
    {
        if (indices_.size() == 0)
            for (int i =0 ; i < cloud_->getSize(); ++i)
                indices_.push_back(i);
    }



    GenericCloud * cloud_;
    StratigraphicModelBase * model_;

    std::vector<int> indices_;

    std::vector<float> stratigraphic_positions_;

    float step_;
    float bandwidth_;

    std::string fieldname_;


};


}//end nspace

#endif // STRATIGRAPHIC_LOGGER_H
