#ifndef QGEO_PLOTTER_H
#define QGEO_PLOTTER_H

#include "qcustomplot.h"
#include "MainScale.h"
#include "PlotsContainer.h"
#include <ccOutOfCore/ccTimeSeries.h>


//// main plot widget
class PlotterWidget : public QCustomPlot
{
    Q_OBJECT
public:
    explicit PlotterWidget(QWidget *parent = 0);

    ~PlotterWidget()
    {
        delete m_main_scale;
        delete m_plots_container;
        delete m_group;
    }

    void addSinglePlot(SinglePlot * plot)
    {

        std::cout << "added plot" << std::endl;
        plot->setMarginGroup(QCP::msTop|QCP::msBottom, m_group);

        //also connect the the range to the main plot
        connect(getMainAxisRect()->getMainAxis(), SIGNAL(rangeChanged(QCPRange)), plot->getDepthAxis(), SLOT(setRange(QCPRange)));
        connect(plot->getDepthAxis(), SIGNAL(rangeChanged(QCPRange)), getMainAxisRect()->getMainAxis(), SLOT(setRange(QCPRange)));


        m_plots_container->addSinglePlot(plot);

        getMainAxisRect()->getMainAxis()->setRange(plot->getDepthAxis()->range());
        plot->getGraph()->rescaleAxes(true);

        replot();

    }

    MainScale * getMainAxisRect() const
    {
        return m_main_scale;
    }



    PlotsContainer * getPlotContainer()
    {
        return m_plots_container;
    }


public slots:
    void handleNewTimeSeries(ccTimeSeries *series);

    void saveAllSeries();



protected:
    MainScale * m_main_scale;

    PlotsContainer * m_plots_container;

    /// this this is the group each axisrect will be forced to belong
    QCPMarginGroup *m_group;




};

#endif // QGEO_PLOTTER_H
