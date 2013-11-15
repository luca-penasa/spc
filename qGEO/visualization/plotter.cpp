#include "plotter.h"




Plotter::Plotter(QWidget *parent): QCustomPlot(parent), m_main_scale(0)
{
    //clear the layout
    plotLayout()->clear(); //clear all

    QCPLayoutGrid *mainLayoutGrid = new QCPLayoutGrid();
    plotLayout()->addElement(0, 0, mainLayoutGrid);

    m_main_scale = new  MainScale(this);
    mainLayoutGrid->addElement(0,0,m_main_scale);

    replot();

    //        addRandomContinousValuesLog();

    //        std::cout << "INSTANTIATED" << std::endl;



    //    CPAxisRect* r = new QCPAxisRect(plot);



    //    m_plot = this->qwtPlot;

    //    m_plot->setAutoReplot(true);
    //    // panning with the left mouse button
    //    (void) new QwtPlotPanner( m_plot->canvas() );

    //    // zoom in/out with the wheel
    //    (void) new QwtPlotMagnifier( m_plot->canvas() );

//    setWindowFlags( Qt::Dialog & Qt::WindowMinMaxButtonsHint);

    //    this->setVisible(true);
    //this->setEnabled(true);
    //    this->open();



    //    connect(this->actionExport, SIGNAL(triggered()), SLOT(exportDocument()));
    //    connect(this->actionSaveTimeSeries, SIGNAL(triggered()), SLOT(saveCurve()));
    //    connect(this->actionClearPlot, SIGNAL(triggered()), SLOT(clearPlot()));

    //    m_current_color = 6;
}
