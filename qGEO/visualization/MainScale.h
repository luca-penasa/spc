#ifndef QGEO_MAINSCALE_H
#define QGEO_MAINSCALE_H

#include "qcustomplot.h"

class MainScale: public QCPAxisRect
{
    Q_OBJECT
public:   
    MainScale(QCustomPlot *parentPlot);

    QCPAxis * getMainAxis() const;
protected:
    QCPAxis * m_main_axis;
};

#endif // QGEO_MAINSCALE_H
