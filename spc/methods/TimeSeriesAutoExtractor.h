#ifndef TIMESERIESAUTOEXTRACTOR_H
#define TIMESERIESAUTOEXTRACTOR_H

#include <spc/elements/VirtualOutcrop.h>
#include<spc/elements/SelectionRubberband.h>
#include <spc/elements/CloudDataSourceOnDisk.h>
#include <spc/methods/TimeSeriesGenerator.h>

namespace spc
{

//! extract all the time series it can find in an outcrop!
class TimeSeriesAutoExtractor
{
public:
    TimeSeriesAutoExtractor();

    void setRoot(spc::ElementBase::Ptr root)
    {
        root_ = root;
    }



    void compute()
    {

        clouds_ =  root_->findElementsThatAre<spc::CloudDataSourceOnDisk>(&spc::CloudDataSourceOnDisk::Type);



        int counter= 0;
        for (spc::CloudDataSourceOnDisk::Ptr cloud: clouds_)
        {
            // the parent MUST be a cloud
            LOG(INFO) << "Processing cloud: "<< cloud->getElementName() << ". -> number " << counter << " of " <<clouds_.size();

            LOG(INFO) << "Looking for selections in childs";

            std::vector<spc::SelectionRubberband::Ptr> selections = cloud->findElementsThatAre<spc::SelectionRubberband>(&spc::SelectionRubberband::Type);

            LOG(INFO) << "Found " <<  selections.size()  << " selections.";

            if (selections.size() == 0)
            {
                LOG(WARNING) << "no selection for cloud " << cloud->getElementName();
                continue;
            }


            LOG(INFO) << "loading the cloud";
            PointCloudBase::Ptr loaded_cloud = cloud->load();
            LOG(INFO) << "loading done.";

            // now each selection must be under a stratigraphic model somewhere in the parents
            for (SelectionRubberband::Ptr sel: selections)
            {

                std::vector<spc::StratigraphicModelBase::Ptr> models = sel->findElementsInParents<spc::StratigraphicModelBase>(&StratigraphicModelBase::Type);

                LOG(INFO) << "the selection is under " << models.size() << " models";
                if (models.size() > 1)
                {
                    LOG(WARNING) << "we are gonna use only the first model found";
                }
                else if (models.size() == 0)
                {
                    LOG(WARNING) << "No stratigraphic model found for the current selection. Going ahead with the next selection";
                    continue;
                }

                spc::StratigraphicModelBase::Ptr model = models.at(0);

                LOG(INFO) << "everything is ok.... computing the time series";


                spc::TimeSeriesGenerator generator;

                generator.setInputCloud(loaded_cloud);
                generator.setSelection(sel);
                generator.setDoAutoCalibration(true);
                generator.setStratigraphicModel(model);
                generator.setBandwidth(bandwidth_);
                generator.setSamplingStep(sampling_step_);
                generator.setYFieldName(scalar_field_name_);
                generator.compute();

                LOG(INFO) << "computing done";
                TimeSeriesEquallySpaced::Ptr outserie =  generator.getOutputSeries();

                if (outserie)
                    sel->addChild(outserie);



            }





            counter++;
        }
    }




protected:

    std::vector<spc::CloudDataSourceOnDisk::Ptr> clouds_;
    spc::ElementBase::Ptr root_;

    float sampling_step_ = 0.01;
    float bandwidth_ = 0.01;

    std::string scalar_field_name_ = "intensity";

};
}

#endif // TIMESERIESAUTOEXTRACTOR_H
