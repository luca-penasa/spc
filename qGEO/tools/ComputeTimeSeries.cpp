#include "ComputeTimeSeries.h"

//#include "kernel_smoothing.h"
#include <ccPointCloud.h>
  
#include <iostream>
#include <fstream>

#include <ccConsole.h>
#include <ccProgressDialog.h>
#include <GenericProgressCallback.h>

#include <ccHObjectCaster.h>

//#include <dialogs/ccCurvePlotterDlg.h>

#include <spc/methods/linear_interpolator.h>
#include <spc/time_series/sparse_time_series.h>

#include <spc/methods/time_series_generator.h>
#include <spc/io/pointcloud2_reader.h>
#include <spc/methods/cloud_slicer.h>

#include <math.h>

#include <qGEO/qGEO.h>

ComputeTimeSeries::ComputeTimeSeries(ccPluginInterface *parent_plugin) : BaseFilter(FilterDescription(   "Compute Time Series",
                                                                         "Compute Time Series",
                                                                         "Build a time series that describe how some scalar change along a given direction/stratigraphic position",
                                                                         ":/toolbar/icons/time_series.png"), parent_plugin)
{
}

int ComputeTimeSeries::checkSelected()
{
    //In most of the cases we just need 1 CC_POINT_CLOUD
    if (m_selected.empty())
        return -11;

    ccHObject::Container entities;
    getAllEntitiesOfType(CC_POINT_CLOUD, entities);

    if (entities.empty())
        return -11;

    return 1;
}

int ComputeTimeSeries::compute()
{
    computed_series.clear();

    ccHObject::Container entities;
    getSelectedEntitiesThatAreCCPointCloud(entities);

    std::cout << "We got n " << entities.size() << std::endl;

    int sf_x = m_dialog->getSFIdA();
    int sf_y = m_dialog->getSFIdB();

    std::cout << "id1 " << sf_x << std::endl;
    std::cout << "id2 " << sf_y << std::endl;

    ComboItemDescriptor x_desc = m_dialog->comboScalars->itemData(sf_x).value<ComboItemDescriptor>();
    ComboItemDescriptor y_desc = m_dialog->comboScalars_2->itemData(sf_y).value<ComboItemDescriptor>();

    std::string x_fname = x_desc.name.toStdString();
    std::string y_fname = y_desc.name.toStdString();

    std::cout << "x name " << x_fname << std::endl;
    std::cout << "y name " << y_fname << std::endl;



    float step = m_dialog->getStep();
    float bandwidth = m_dialog->getBandwidth();

    //pre compute min max of the field, so we can build time series all with the same extension!
    float overall_max = -1e100;
    float overall_min = 1e100;
    for (ccHObject *obj: entities)
    {
        ccPointCloud * cloud = ccHObjectCaster().ToPointCloud(obj);
        int sf_id = cloud->getScalarFieldIndexByName(x_fname.c_str());
        CCLib::ScalarField * field = cloud->getScalarField(sf_id);
        float max =  field->getMax();
        float min = field->getMin();
        if (max > overall_max)
            overall_max = max;

        if (min < overall_min)
            overall_min = min;
    }


    std::cout << "x min " << overall_min << std::endl;
    std::cout << "x max " << overall_max << std::endl;

    for (ccHObject *obj: entities)
    {
        ccPointCloud * cloud = ccHObjectCaster().ToPointCloud(obj);
        spc::PointCloud2Reader * reader = new spc::PointCloud2Reader(cloud);

        spc::TimeSeriesGenerator<float> generator;
        generator.setInputReader(reader);
        generator.setFixedMinMax(overall_min, overall_max);
        generator.setBandwidth(bandwidth);
        generator.setSamplingStep(step);
        generator.setXFieldName(x_fname);
        generator.setYFieldName(y_fname);

        generator.compute();

        computed_series.push_back(generator.getOutputSeries());
    }
    return 1;
}


int ComputeTimeSeries::openInputDialog()
{
    if (!m_dialog)
        m_dialog = new ComputeTimeSeriesDlg;

    ccPointCloud * cloud = getSelectedEntityAsCCPointCloud();

    if (!cloud)
        return -1;

    m_dialog->updateComboScalars(cloud);

    return m_dialog->exec() ? 1 : 0;
}

int ComputeTimeSeries::openOutputDialog()
{

//    qGEO * plugin = static_cast<qGEO *>(getParentPlugin());

//    ccCurvePlotterDlg * plotter = plugin->getCurrentPlotter();

//    if (!plotter)
//        return -1;

//    else if (!m_dialog->getAppendPlot())
//        plotter->clearPlot();


//    for (auto series: computed_series)
//    {
//        if (series.getY().size() != 0)
//            plotter->addCurve(series);
//    }


//    plotter->show();
//    plotter->raise();

//    return 1;

}

