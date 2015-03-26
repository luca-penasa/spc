#ifndef CALIBRATIONMODELBASE_H
#define CALIBRATIONMODELBASE_H
#include <spc/elements/PointCloudBase.h>

namespace  spc {




class SPC_LIB_API  CalibrationModelBase
{
public:
    CalibrationModelBase();

    virtual Eigen::VectorXf getCorrectedIntensity(spc::PointCloudBase::Ptr cloud, const std::vector<size_t> ids) = 0;

};



}// end nspace
#endif // CALIBRATIONMODELBASE_H
