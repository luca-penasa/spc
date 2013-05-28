#include "ComputeTimeSeries.h"

//#include "kernel_smoothing.h"
#include <ccPointCloud.h>
  
#include <iostream>
#include <fstream>

#include <ccConsole.h>
#include <ccProgressDialog.h>
#include <GenericProgressCallback.h>

#include <dialogs/ccCurvePlotterDlg.h>

#include <spc/methods/linear_interpolator.h>
#include <spc/time_series/sparse_time_series.h>



//#include <lidarlib.h>

#include <math.h>

ComputeTimeSeries::ComputeTimeSeries() : BaseFilter(FilterDescription(   "Compute Time Series",
                                                                         "Compute Time Series",
                                                                         "Build a time series that describe how some scalar change along a given direction/stratigraphic position",
                                                                         ":/toolbar/icons/time_series.png")), m_dialog(0), m_plot_dialog(0)
{
}

int ComputeTimeSeries::compute()
{
    ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
    if (!cloud) //just to be sure
      return -1;
    
    int sf_x = m_dialog->getSFIdA();
    int sf_y = m_dialog->getSFIdB();
  
    ComboItemDescriptor x_desc = m_dialog->comboScalars->itemData(sf_x).value<ComboItemDescriptor>();
    ComboItemDescriptor y_desc = m_dialog->comboScalars_2->itemData(sf_y).value<ComboItemDescriptor>();
    
    
    vType x = getComboItemAsStdFloatVector(x_desc, cloud); //strat pos, normally
    vType y = getComboItemAsStdFloatVector(y_desc, cloud);
    
        

    float x_min = *min_element(x.begin(), x.end());
    float x_max = *max_element(x.begin(), x.end());
    float step = m_dialog->getStep();

    spc::EquallySpacedTimeSeries<nType> * series = new  spc::EquallySpacedTimeSeries<nType>(x_min, x_max, step);
    auto new_x = series->getX();

    spc::SparseTimeSeries<nType> *orig_data = new spc::SparseTimeSeries<nType>(x,y);

    float bandwidth = m_dialog->getBandwidth();

    spc::KernelSmoothing<nType> * ks = new spc::KernelSmoothing<nType>;

    ks->setInput(orig_data);
    ks->setBandwidth(bandwidth);
//    ks->setEvaluationPositions(new_x);
    ks->setUseWeights(false);
    ks->compute(series);
    vType new_y = series->getY();


    ccScalarField * field1 = new ccScalarField;
    field1->resize(cloud->size());
    field1->setName("intial model_int");


    spc::LinearInterpolator<nType> interpolator;
    interpolator.setNewX(x);
    interpolator.setXY(new_y, step, x_min);
    interpolator.compute();
    vType model_int_init = interpolator.getNewY(); //intensities from the model

    for (int i = 0; i < cloud->size(); ++i)
    {
        field1->setValue(i, model_int_init[i]);
    }

    field1->computeMinAndMax();

    cloud->addScalarField(field1);




    if (m_dialog->getIterativeReweighting())
    { //we redo the same thing more times using weights!
        //but extract as vType the scalar field representing Stratigraphic Position, for each point in cloud

        for (size_t iteration = 0 ; iteration < m_dialog->getNReweightingIterations(); ++iteration )
        {


            std::cout << "initial new_y:!" << std::endl;
            for (int i = 100; i < 120; ++i)
                std::cout << new_y[i] << std::endl;

            vType model_int;
            //we use a nn_intepolator to get the int value for each point
            spc::LinearInterpolator<nType> interpolator;
            interpolator.setNewX(x);
            interpolator.setXY(new_y, step, x_min);
            interpolator.compute();
            model_int = interpolator.getNewY(); //intensities from the model

            ccScalarField * field2 = new ccScalarField;
            field2->resize(cloud->size());
            field2->setName("after model_int");

            for (int i = 0; i < cloud->size(); ++i)
            {
                field2->setValue(i, model_int[i]);
            }

            field2->computeMinAndMax();

            cloud->addScalarField(field2);




            std::cout << "model_int:!" << std::endl;
            for (int i = 0; i < 10; ++i)
                std::cout << model_int[i] << std::endl;



            //convert compute the difference
            vType diff = spc::get_difference(y, model_int);


            float iter_bandwidth = 20;
            std::transform(diff.begin(), diff.end(), diff.begin(), [&](nType d) -> nType { return std::abs(d)/iter_bandwidth ; }  );

            //transform diffs to weights
            std::transform(diff.begin(), diff.end(), diff.begin(), [&](nType d) -> nType { return gaussian(d) ; }  );

            ccScalarField * field = new ccScalarField;
            field->resize(cloud->size());
            field->setName("weights");




            for (int i = 0; i < cloud->size(); ++i)
            {
                field->setValue(i, diff[i]);
            }

            field->computeMinAndMax();

            cloud->addScalarField(field);


            std::cout << "data:!" << std::endl;
            for (int i = 0; i < 10; ++i)
                std::cout << diff[i] << std::endl;


            spc::KernelSmoothing<nType> * ks2 = new spc::KernelSmoothing<nType>;

            ks2->setXY(x, y);
            ks2->setBandwidth(bandwidth);
//            ks2->setEvaluationPositions(new_x);
            ks2->setWeights(diff);
            ks2->setUseWeights(true);
            ks2->compute(series);
            new_y = series->getY();

            std::cout << "final new_y:!" << std::endl;
            for (int i = 100; i < 120; ++i)
                std::cout << new_y[i] << std::endl;

        }

    }

    //save last plots
    last_x = new_x;
    last_y = new_y;
        
    return 1;

}


int ComputeTimeSeries::openInputDialog()
{
    if (!m_dialog)
        m_dialog = new ComputeTimeSeriesDlg;

    ccPointCloud * cloud = getSelectedEntityAsCCPointCloud();
    m_dialog->updateComboScalars(cloud);

    return m_dialog->exec() ? 1 : 0;
}

int ComputeTimeSeries::openOutputDialog()
{
    if (!m_plot_dialog )
        m_plot_dialog = new ccCurvePlotterDlg;
    else if (!m_dialog->getAppendPlot())
        m_plot_dialog->clearPlot();


    m_plot_dialog->addCurve<float>(last_x, last_y);

    m_plot_dialog->show();
    m_plot_dialog->raise();

    return 1;

}

