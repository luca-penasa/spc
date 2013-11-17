#include "SinglePlot.h"


SinglePlot::SinglePlot(QCustomPlot *parentPlot): QCPAxisRect(parentPlot, false)
{
    init();
}

SinglePlot::SinglePlot(QCustomPlot *parentPlot, spc::ContinousValuesLog *log): QCPAxisRect(parentPlot, false)
{
    init();
    updateDataWith(log);
}

void SinglePlot::updateDataWith(const spc::ContinousValuesLog *log)
{
    //data are copied!
    m_depth = QVector<double>::fromStdVector(log->getStratigraphicPositions<double>());
    m_values = QVector<double>::fromStdVector(log->getValues<double>());

    // set these data into the graph
    updateGraphData();
}

void SinglePlot::init()
{
    setMaximumSize(QSize(250, 1000000000)); // we limit the size in width
    setMinimumSize(120, 120);
    m_bottom_axis = addAxis(QCPAxis::atBottom);
    m_depth_axis = addAxis(QCPAxis::atLeft);

    m_depth_axis->setVisible(false);

    //create a graph
    m_graph = parentPlot()->addGraph(m_depth_axis, m_bottom_axis);
}
