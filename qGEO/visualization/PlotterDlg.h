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



#include <visualization/ui_PlotterDlg.h>

#include <spc/time_series/equally_spaced_time_series.h>
#include <spc/elements/continous_values_stratigraphy.h>



//// te dialog with the widgt collecting the plots
class PlotterDlg : public QDialog, Ui::PlotterDlgUi
{
    Q_OBJECT
public:
    PlotterDlg(QWidget *parent=0);

    void addContinousValuesLog(spc::ContinousValuesLog * log);

    void addRandomContinousValuesLog();


    PlotterWidget * getPlotterWidget()
    {
        return plot;
    }

    PropertiesViewerWidget * getPropertiesWidget()
    {
        return props;
    }


public slots:
    void update()
    {
        this->plot->replot();
    }


protected:

    std::vector<spc::ContinousValuesLog * >   m_continous_logs;

    QCPAxis * m_main_axis;


};









#endif

