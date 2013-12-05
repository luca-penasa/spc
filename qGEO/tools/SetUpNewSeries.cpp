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

#include <ccOutOfCore/ccPlanarSelection.h>

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
    ccHObject * sel_obj = m_dialog->getSelectedArea();


    std::cout << "mod " << mod_obj << std::endl;
    std::cout << "cloud " << cloud_obj << std::endl;


    std::cout << cloud_obj->getName().toStdString().c_str() << std::endl;
    std::cout << mod_obj->getName().toStdString().c_str() << std::endl;

    ccPointCloud * cloud = static_cast<ccPointCloud *> (cloud_obj);


    ccSingleAttitudeModel * model  = static_cast<ccSingleAttitudeModel *> (mod_obj);

    if (!model || !cloud)
        return -1;

    //as shared ptr pointing to a genericloud
    spc::spcGenericCloud::Ptr mycloud ( new CloudWrapper<ccPointCloud>(cloud));

    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

    spc::TimeSeriesGenerator<float> generator;
    std::vector<int> indices;
    if (sel_obj)
    {
        ccPlanarSelection * selection = static_cast<ccPlanarSelection *> (sel_obj);

        selection->setInputCloud(mycloud);
        selection->updateIndices();

        pcl::console::print_warn("found %i points after segmentation", indices.size());

        generator.setIndices(indices);
    }

    // this will create a copy
    spc::spcSingleAttitudeModel::Ptr ptr =  boost::make_shared<spc::spcSingleAttitudeModel>(*model);



    generator.setInputCloud(mycloud);
    generator.setStratigraphicModel(ptr);
    generator.setYFieldName(m_dialog->getSelectedScalarFieldName());
    generator.setSamplingStep(m_dialog->getStep());
    generator.setBandwidth(m_dialog->getBandwidth());


    int status = generator.compute();

    if (status <= 0)
        return -1;


    spc::EquallySpacedTimeSeries<float> ts;
    generator.getOutputSeries(ts);



    ccTimeSeries * series = new ccTimeSeries(ts);


    qGEO::theInstance()->getPlotterDlg()->getPlotterWidget()->handleNewTimeSeries(series);
    emit newEntity(series);



    return 1;
}
