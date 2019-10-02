#include "TimeSeriesAutoExtractor.h"
#include <spc/elements/StratigraphicModelBase.h>

namespace spc
{
TimeSeriesAutoExtractor::TimeSeriesAutoExtractor()
{
}

TimeSeriesEquallySpaced::Ptr TimeSeriesAutoExtractor::extractTimSeries(const PointCloudBase::Ptr loaded_cloud, const SelectionRubberband::Ptr sel, const StratigraphicModelBase::Ptr model)
{
    spc::TimeSeriesGenerator generator;

    generator.setInputCloud(loaded_cloud);
    generator.setSelection(sel);
    generator.setDoAutoCalibration(autocalibrate_);
    generator.setStratigraphicModel(model);
    generator.setBandwidth(bandwidth_);
    generator.setSamplingStep(sampling_step_);
    generator.setYFieldName(scalar_field_name_);
    generator.compute();

    LOG(INFO) << "computing done";
    TimeSeriesEquallySpaced::Ptr outserie = generator.getOutputSeries();

    return outserie;
}

void TimeSeriesAutoExtractor::extractPerSelectionTimeSeries(const PointCloudBase::Ptr loaded_cloud, const std::vector<SelectionRubberband::Ptr> selections)
{
    // now each selection must be under a stratigraphic model somewhere in the parents
    for (SelectionRubberband::Ptr sel : selections) {

        std::vector<spc::StratigraphicModelBase::Ptr> models = sel->findElementsInParents<spc::StratigraphicModelBase>(&StratigraphicModelBase::Type);

        LOG(INFO) << "the selection is under " << models.size() << " models";
        if (models.size() > 1) {
            LOG(WARNING) << "we are gonna use only the first model found";
        }
        else if (models.size() == 0) {
            LOG(WARNING) << "No stratigraphic model found for the current selection. Going ahead with the next selection";
            continue;
        }

        spc::StratigraphicModelBase::Ptr model = models.at(0);

        LOG(INFO) << "everything is ok.... computing the time series";


        auto outseries  = this->extractTimSeries(loaded_cloud,sel, model);


        if (outseries)
            sel->addChild(outseries);
    }
}

void TimeSeriesAutoExtractor::compute()
{

    clouds_ = root_->findElementsThatAre<spc::CloudDataSourceOnDisk>(&spc::CloudDataSourceOnDisk::Type);


    LOG(INFO) <<  "We got  "<< clouds_.size() << " clouds on disk.";

    size_t counter = 0;
    for (spc::CloudDataSourceOnDisk::Ptr cloud : clouds_) {

        // the parent MUST be a cloud
        LOG(INFO) << "Processing cloud: " << cloud->getElementName() << ". -> number " << counter++ << " of " << clouds_.size();
        LOG(INFO) << "Looking for selections in childs";

        std::vector<spc::SelectionRubberband::Ptr> selections = cloud->findElementsThatAre<spc::SelectionRubberband>(&spc::SelectionRubberband::Type);

        LOG(INFO) << "Found " << selections.size() << " selections.";
        if (selections.size() == 0) {
            LOG(WARNING) << "no selection for cloud " << cloud->getElementName();
            continue;
        }

        PointCloudBase::Ptr loaded_cloud = cloud->load();
        extractPerSelectionTimeSeries(loaded_cloud, selections);

    }

    LOG(INFO) <<  "We got  "<< clouds_in_memory_.size() << " clouds yet in memory.";

    counter = 0;
    for (PointCloudBase::Ptr cloud : clouds_in_memory_) {

        std::vector<spc::SelectionRubberband::Ptr> selections = cloud->findElementsThatAre<spc::SelectionRubberband>(&spc::SelectionRubberband::Type);

        LOG(INFO) << "Found " << selections.size() << " selections.";
        if (selections.size() == 0) {
            LOG(WARNING) << "no selection for cloud " << cloud->getElementName();
            continue;
        }

        extractPerSelectionTimeSeries(cloud, selections);

    }

}

}
