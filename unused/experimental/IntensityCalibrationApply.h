#pragma once
#ifndef INTENSITYCALIBRATIONAPPLY_H
#define INTENSITYCALIBRATIONAPPLY_H

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/EigenFunctionBase.h>

#include <spc/methods/TransferFieldNN.h>
namespace spc
{
//! apply a given function to some scalar fields of the clouds.
class ScalarFieldsCalcuator
{
public:
    spcTypedefSharedPtrs(ScalarFieldsCalcuator)

    ScalarFieldsCalcuator();


    int compute();


    void setInputCloud( PointCloudBase::Ptr c)
    {
        cloud_ = c;
    }

    void setFunction(const EigenFunctionBase::Ptr cal_function)
    {
        function_ = cal_function;
    }

    void setFieldsToUse(std::vector<std::string> names)
    {
        if (!function_ )
        {
            PCL_ERROR("Function not given.\n");
            return;
        }

        if (!function_ || names.size() != function_->getInputSize() )
        {
            PCL_ERROR("Fields names and number of parameters of the function do not match\n");
            return;
        }

        fields_to_use_ = names;
    }

    bool areFieldsPresent() const
    {
        int n_good = std::count_if(fields_to_use_.begin(), fields_to_use_.end(), [&](std::string n){return cloud_->hasField(n);});

        return n_good == function_->getInputSize();
    }

    void setOutFieldName(const std::string name)
    {
        out_field_name_ = name;
    }


protected:
    PointCloudBase::Ptr cloud_;

    EigenFunctionBase::Ptr function_;

    std::vector<std::string> fields_to_use_;

    std::string out_field_name_ = "scalar_field";
};

} // end nspace

#endif // INTENSITYCALIBRATIONAPPLY_H
