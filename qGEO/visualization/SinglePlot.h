#ifndef SINGLEPLOT_H
#define SINGLEPLOT_H
#include "qcustomplot.h"
#include <MainScale.h>

#include <spc/elements/continous_values_stratigraphy.h>


class SinglePlot : public QCPAxisRect
{
public:
    SinglePlot(QCustomPlot *parentPlot);

    SinglePlot(QCustomPlot *parentPlot , spc::ContinousValuesLog *log);

    void updateDataWith(const spc::ContinousValuesLog * log);

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


private:
    void updateGraphData()
    {
        m_graph->setData(m_depth, m_values);
        m_graph->rescaleAxes();
    }


    void init();




protected:
    QCPAxis * m_bottom_axis;
    QCPAxis * m_depth_axis;

    QVector<double> m_depth;
    QVector<double> m_values;

    QCPGraph * m_graph;

};

#endif // SINGLEPLOT_H
