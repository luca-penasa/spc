#include "PlotterWidget.h"
#include "PlotsContainer.h"




PlotterWidget::PlotterWidget(QWidget *parent): QCustomPlot(parent), m_main_scale(0), m_plots(0)
{
    //clear the layout
    plotLayout()->clear(); //clear all

    /// a main scale object at LEFT
    m_main_scale = new  MainScale(this);
    plotLayout()->addElement(0,0,m_main_scale);

    /// an area with plots at RIGHT
    m_plots = new PlotsContainer();
    plotLayout()->addElement(0,1, m_plots);

    /// a group for aligning graphs
    m_group = new QCPMarginGroup(this);
    getMainAxisRect()->setMarginGroup(QCP::msTop|QCP::msBottom, m_group);


    setInteractions(QCP::iRangeDrag | QCP::iRangeZoom |QCP::iSelectAxes);

    SinglePlot * sp =  new SinglePlot(this);

    spc::ContinousValuesLog * log =  new spc::ContinousValuesLog;
    log->resize(100);
    for(int i = 0 ; i < log->getSize(); ++i)
        log->setValue(i, i);

    sp->updateDataWith(log);
    addSinglePlot(sp);




    replot();

}
