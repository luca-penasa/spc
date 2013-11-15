#ifndef COMPUTETIMESERIES_H
#define COMPUTETIMESERIES_H

//#include <dialogs/ccCurvePlotterDlg.h>
#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <spc/methods/kernel_smoothing.h>
#include <dialogs/ComputeTimeSeriesDlg.h>





class ComputeTimeSeries: public BaseFilter
{
    Q_OBJECT
    typedef float nType;
    typedef std::vector<nType> vType;

public:
    ComputeTimeSeries(ccPluginInterface * parent_plugin = 0);

    //compute gaussian weights on a single numeric value
    inline nType
    gaussian(const nType &value)
    {
        return 1.0/sqrt(2.0*M_PI) * exp(-0.5*value*value);
    }

protected:
   int compute();
   int openInputDialog();
   int openOutputDialog();

   virtual int checkSelected();

   ComputeTimeSeriesDlg * m_dialog;
//   ccCurvePlotterDlg * m_plot_dialog;

   std::vector<spc::EquallySpacedTimeSeries<float> > computed_series;

};



#endif // COMPUTETIMESERIES_H





