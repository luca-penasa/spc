#include "SetUpNewSeries.h"
#include <ccArrow.h>

#include <dialogs/ccTimeSeriesGeneratorEditorDlg.h>

//#include <ccOutOfCore/ccMyCCHObject.h>
//#include <ccOutOfCore/ccAdditionalCaster.h>

//#include <spc/geology/stratigraphic_model_base.h>
#include <ccOutOfCore/ccSingleAttitudeModel.h>

#include <ccHObjectCaster.h>
#include <CloudMapper.h>

#include <iostream>
#include <qPCL/PclUtils/utils/cc2sm.h>

#include <spc/elements/generic_cloud.h>

#include <spc/methods/time_series_generator.h>

#include <ccOutOfCore/ccTimeSeriesGenerator.h>

#include <qGEO/qGEO.h>
SetUpNewSeries::SetUpNewSeries(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Add a new time series generator",
                                                                                                   "Add a new time series generator",
                                                                                                   "Add a new time series generator",
                                                                                                   ":/toolbar/icons/tseries_generator.png")
                                                                              , parent_plugin)
{
    this->setShowProgressBar(false);
}

int SetUpNewSeries::compute()
{
    ccTimeSeriesGenerator * generator =  new ccTimeSeriesGenerator();

    newEntity(generator);



//    //get the objects from the combo
//    ccHObject * mod_obj = m_dialog->getSelectedModel();
//    ccHObject * cloud_obj = m_dialog->getSelectedCloud();

//    if (!mod_obj || !cloud_obj)
//        return -1;

//    ccPointCloud * cloud = static_cast<ccPointCloud *> (cloud_obj);
//    ccSingleAttitudeModel * model = static_cast<ccSingleAttitudeModel *> (mod_obj);

//    //try the mapper
//    spc::GenericCloud * mycloud = new CloudWrapper<ccPointCloud>(cloud);

//    spc::TimeSeriesGenerator<float> generator;
//    generator.setInputCloud(mycloud);
//    generator.setStratigraphicModel(model);
//    generator.setYFieldName(m_dialog->getSelectedScalarFieldName());
//    generator.setSamplingStep(m_dialog->getStep());
//    generator.setBandwidth(m_dialog->getBandwidth());
//    int status = generator.compute();

//    if (status <= 0)
//        return -1;

//    spc::EquallySpacedTimeSeries<float> ts = generator.getOutputSeries();


//    qGEO * plugin = static_cast<qGEO *> (getParentPlugin());

//    PlotterWidget * plotter = plugin->getPlotterDlg()->getPlotterWidget();

//    SinglePlot * plot = new SinglePlot(plotter);
//    plot->updateDataWith(ts);
//    plotter->addSinglePlot(plot);



    return 1;
}





int SetUpNewSeries::checkSelected()
{
    //always active!
    return 1;
}
