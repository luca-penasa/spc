#ifndef QGEO_PLOTTER_H
#define QGEO_PLOTTER_H

#include "qcustomplot.h"
#include "MainScale.h"

class Plotter : public QCustomPlot
{
    Q_OBJECT
public:
    explicit Plotter(QWidget *parent = 0);
protected:
    MainScale * m_main_scale;


};

#endif // QGEO_PLOTTER_H
