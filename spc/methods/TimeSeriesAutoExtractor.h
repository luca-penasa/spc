#ifndef TIMESERIESAUTOEXTRACTOR_H
#define TIMESERIESAUTOEXTRACTOR_H

#include <spc/elements/VirtualOutcrop.h>
#include <spc/elements/SelectionRubberband.h>
#include <spc/elements/CloudDataSourceOnDisk.h>
#include <spc/methods/TimeSeriesGenerator.h>




namespace spc {

spcFwdDeclSharedPtr(StratigraphicModelBase)

//! extract all the time series it can find in an outcrop!
class TimeSeriesAutoExtractor {
public:
    TimeSeriesAutoExtractor();

    void setRoot(spc::ElementBase::Ptr root)
    {
        root_ = root;

        LOG(INFO) << "Root object " << root->getElementName();
    }

    // shortcut to extract ts
    TimeSeriesEquallySpaced::Ptr extractTimSeries(const PointCloudBase::Ptr loaded_cloud,
                          const SelectionRubberband::Ptr sel,
                          const spc::StratigraphicModelBasePtr model);


    //! each ts will be set as child to its own selection
    void extractPerSelectionTimeSeries(const PointCloudBase::Ptr loaded_cloud,
                           const std::vector<SelectionRubberband::Ptr> selections);


    void compute();


protected:
    std::vector<spc::CloudDataSourceOnDisk::Ptr> clouds_;

    std::vector<spc::PointCloudBase::Ptr> clouds_in_memory_;
    spc::ElementBase::Ptr root_;

    float sampling_step_ = 0.01;
    float bandwidth_ = 0.01;

    bool autocalibrate_ = false;

    std::string scalar_field_name_ = "intensity";
};
}

#endif // TIMESERIESAUTOEXTRACTOR_H
