//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################



#ifndef CC_2DPLOT_WINDOW_HEADER
#define CC_2DPLOT_WINDOW_HEADER



#include <ui_curvePlotterDlg.h>

#include <spc/time_series/base_time_series.h>

class QwtPlot;
class QwtPlotCurve;

class ccCurvePlotterDlg : public QDialog, Ui::CurvePlotterDialog
{
    Q_OBJECT
public:
    ccCurvePlotterDlg(QWidget *parent=0);

    template <class sType>
    void addCurve(std::vector<sType> &x, std::vector<sType> &y);

    template <typename sType>
    void addCurve(spc::GenericTimeSeries<sType> * tseries);


    void callReplot();
    //enum colors {red=7, green=8, blue=9, cyan=10, magenta=11, yellow=12, darkRed=13, darkGreen=14, darkBlue=15, darkCyan=16, darkMagenta=17, darkYellow=18 };


public slots:
    void exportDocument();
    void saveCurve();
    void clearPlot();

protected:
    QColor nextColor();

protected:
    QwtPlot * m_plot;

    std::vector<QwtPlotCurve *> m_curves;
    std::vector<QVector<QPointF> *> m_vectors;

    int m_current_color;
};









#endif

