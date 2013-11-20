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
#include <ccOutOfCore/ccTimeSeries.h>
#include <qGEO/qGEO.h>

#include <PlotterWidget.h>
#include <PlotterDlg.h>
#include <OpenPlotsDialog.h>

SetUpNewSeries::SetUpNewSeries(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Add a new time series generator",
                                                                                                   "Add a new time series generator",
                                                                                                   "Add a new time series generator",
                                                                                                   ":/toolbar/icons/tseries_generator.png")
                                                                              , parent_plugin), m_dialog(0)

{
    this->setShowProgressBar(false);

    PlotterWidget * plotter = qGEO::theInstance()->getPlotterDlg()->getPlotterWidget();
//    connect(this, SIGNAL(newSeries(ccTimeSeries)), plotter, SLOT(handleNewTimeSeries(ccTimeSeries)));

}

int SetUpNewSeries::compute()
{
//    ccTimeSeriesGenerator * generator =  new ccTimeSeriesGenerator();

//    newEntity(generator);

    return 1;
}


int SetUpNewSeries::checkSelected()
{
    //always active!
    return 1;
}

int SetUpNewSeries::openInputDialog()
{
    if (!m_dialog)
        m_dialog = new ComputeTimeSeriesDlg(0);

    m_dialog->initWithTree();

    return m_dialog->exec() ? 1 : 0;
}

int SetUpNewSeries::openOutputDialog()
{

    //get the objects from the combo
    ccHObject * mod_obj = m_dialog->getSelectedModel();
    ccHObject * cloud_obj = m_dialog->getSelectedCloud();

    if (!mod_obj || !cloud_obj)
        return -1;

    ccPointCloud * cloud = static_cast<ccPointCloud *> (cloud_obj);
    ccSingleAttitudeModel * model = static_cast<ccSingleAttitudeModel *> (mod_obj);

    //try the mapper
    spc::GenericCloud * mycloud = new CloudWrapper<ccPointCloud>(cloud);

    spc::TimeSeriesGenerator<float> generator;
    generator.setInputCloud(mycloud);
    generator.setStratigraphicModel(model);
    generator.setYFieldName(m_dialog->getSelectedScalarFieldName());
    generator.setSamplingStep(m_dialog->getStep());
    generator.setBandwidth(m_dialog->getBandwidth());
    int status = generator.compute();

    if (status <= 0)
        return -1;


    spc::EquallySpacedTimeSeries<float> ts = generator.getOutputSeries();

    ccTimeSeries * series = new ccTimeSeries(ts);
    series->setEnabled(false);

    std::cout << "-1" << std::endl;
    std::cout << qGEO::theInstance()->getPlotterDlg()->getPlotterWidget() << std::endl;
    SinglePlot * newplot = new SinglePlot(qGEO::theInstance()->getPlotterDlg()->getPlotterWidget());

    std::cout << "1" << std::endl;
    newplot->updateDataWith(*series);
std::cout << "2" << std::endl;
    PlotterDlg * plotter = qGEO::theInstance()->getPlotterDlg();
    std::cout << "3" << std::endl;
    plotter->getPlotterWidget()->addSinglePlot(newplot);
std::cout << "4" << std::endl;
    emit newEntity(series);
return 1;
}
