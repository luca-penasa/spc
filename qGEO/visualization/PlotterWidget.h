#ifndef QGEO_PLOTTER_H
#define QGEO_PLOTTER_H

#include "qcustomplot.h"
#include "MainScale.h"
#include "PlotsContainer.h"


//// main plot widget
class PlotterWidget : public QCustomPlot
{
    Q_OBJECT
public:
    explicit PlotterWidget(QWidget *parent = 0);

    void addSinglePlot(SinglePlot * plot)
    {


        plot->setMarginGroup(QCP::msTop|QCP::msBottom, m_group);

        //also connect the the range to the main plot
        connect(getMainAxisRect()->getMainAxis(), SIGNAL(rangeChanged(QCPRange)), plot->getDepthAxis(), SLOT(setRange(QCPRange)));
        connect(plot->getDepthAxis(), SIGNAL(rangeChanged(QCPRange)), getMainAxisRect()->getMainAxis(), SLOT(setRange(QCPRange)));


        m_plots->addSinglePlot(plot);

        getMainAxisRect()->getMainAxis()->setRange(plot->getDepthAxis()->range());
        plot->getGraph()->rescaleAxes(true);

        replot();

    }

    MainScale * getMainAxisRect() const
    {
        return m_main_scale;
    }







protected:
    MainScale * m_main_scale;

    PlotsContainer * m_plots;

    /// this this is the group each axisrect will be forced to belong
    QCPMarginGroup *m_group;




};

#endif // QGEO_PLOTTER_H
