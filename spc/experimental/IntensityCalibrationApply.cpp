#include "IntensityCalibrationApply.h"
namespace spc
{
ScalarFieldsCalcuator::ScalarFieldsCalcuator()
{
}

int ScalarFieldsCalcuator::compute()
{

    PCL_DEBUG("[ScalarFieldsCalcuator] starting intensity calibration compute()\n");

    if (!function_)
    {
        PCL_ERROR("[ScalarFieldsCalcuator]  Cannot Compute, calibration function void\n");
        return -1;
    }

    if (!areFieldsPresent())
    {
        PCL_ERROR("[ScalarFieldsCalcuator] Requested Fields Are not Present\n");
        return -1;
    }


    PCL_DEBUG("getting the fields...\n");

    Eigen::MatrixXf X;
    X.resize(cloud_->getNumberOfPoints(),  function_->getInputSize());

    Eigen::MatrixXf Y;
    Y.resize(cloud_->getNumberOfPoints(), function_->getOutputSize());


    // create the input matrix
    int counter = 0;
    for (std::string fname: fields_to_use_)
    {
        std::vector<float> v = cloud_->getField(fname);
        Eigen::Map<Eigen::VectorXf> v_eig(v.data(), v.size());
        X.col(counter++) = v_eig;
    }


    // co computations
    Y = function_->operator() (X);

    for (int i = 0; i < function_->getOutputSize(); ++i)
    {
          cloud_->addField(out_field_name_ + "@" + boost::lexical_cast<std::string>(i));
    }


    for (int j = 0; j < function_->getOutputSize(); ++j)
    {
        std::string f_name = out_field_name_ + "@" + boost::lexical_cast<std::string>(j);

        for (int i = 0; i < Y.rows(); ++i)
            cloud_->setFieldValue(i, out_field_name_, Y(i) );
    }
}
}
