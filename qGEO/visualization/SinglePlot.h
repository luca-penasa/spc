#ifndef SINGLEPLOT_H
#define SINGLEPLOT_H
#include "qcustomplot.h"
#include <MainScale.h>

#include <spc/elements/continous_values_stratigraphy.h>
#include <spc/time_series/base_time_series.h>
#include <ccOutOfCore/ccTimeSeries.h>

class SinglePlot : public QCPAxisRect
{
public:
    SinglePlot(QCustomPlot *parentPlot);

    /// DEPRECATED
    SinglePlot(QCustomPlot *parentPlot , spc::ContinousValuesLog *log);

    /// DEPRECATED
    void updateDataWith(const spc::ContinousValuesLog * log);

    void updateDataWith(ccTimeSeries * tserie);

    QCPAxis * getDepthAxis()
    {
        return m_depth_axis;
    }

    QCPGraph * getGraph () const
    {
        return m_graph;
    }

    void setDepthRange(const float lower, const float upper)
    {
        getDepthAxis()->setRange(lower, upper);
    }

    void connectToMainAxis(MainScale * main)
    {
        connect(this, SIGNAL(rangeChanged(QCPRange)), main->getMainAxis(), SLOT(setRange(QCPRange)));
        connect(main->getMainAxis(), SIGNAL(rangeChanged(QCPRange)), this, SLOT(setRange(QCPRange)));

    }


    ccTimeSeries * getTimeSeries()
    {
        return m_tseries;
    }


private:
    void updateGraphData(QVector<double> x, QVector<double> y)
    {
        m_graph->setData(x,y);
        m_graph->rescaleAxes();
    }




    ccTimeSeries * m_tseries;




    void init();




protected:
    QCPAxis * m_bottom_axis;
    QCPAxis * m_depth_axis;

    QCPGraph * m_graph;

};

#endif // SINGLEPLOT_H
