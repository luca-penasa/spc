#ifndef QGEO_PLOTSCONTAINER_H
#define QGEO_PLOTSCONTAINER_H

#include "qcustomplot.h"

#include "SinglePlot.h"

/// a cointainer for gological plots!
class PlotsContainer: public QCPLayoutGrid
{
    Q_OBJECT
public:
    explicit PlotsContainer();

    void addSinglePlot(SinglePlot * plot)
    {
        addElement(0, getNumberOfPlots(), plot);
        m_plots.push_back(plot); // keep a ref here
    }
    int getNumberOfPlots() const
    {
        return m_plots.size();
    }

    SinglePlot * getSinglePlot(int id) const
    {
        assert(id < m_plots.size());
        return m_plots.at(id);
    }


    void setDepthRangeForAllPlots(const float lower, const float upper)
    {
        for (SinglePlot * plot: m_plots)
            plot->setDepthRange(lower, upper);
    }

protected:
    std::vector<SinglePlot *> m_plots;

};

#endif // QGEO_PLOTSCONTAINER_H
