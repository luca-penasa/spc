#ifndef Q_PCL_PLUGIN_COMPUTETIMESERIES_HEADER
#define Q_PCL_PLUGIN_COMPUTETIMESERIES_HEADER

#include <ui_ComputeTimeSeriesDlg.h>
#include <ccPointCloud.h>
#include <spc/methods/kernel_smoothing.h>

class ComputeTimeSeriesDlg : public QDialog, public Ui::ComputeTimeSeriesDialog
{
    Q_OBJECT
public:
    ComputeTimeSeriesDlg(QWidget* parent=0);

    double getBandwidth() const { return this->spinBandwidth->value(); }
    double getStep() const { return this->spinStep->value(); }

    int getSFIdA() const { return this->comboScalars->currentIndex(); }
    int getSFIdB() const { return this->comboScalars_2->currentIndex(); }

    bool getProduceRandomizedSeries() const { return this->groupBoxRandomized->isChecked(); }
    bool getProduceCrossRandomizedSeries() const { return this->groupBoxCrossRandomized->isChecked(); }

    bool getAppendPlot() const {return this->checkBoxAppendPlot->isChecked();}

    void updateComboScalars(const ccPointCloud * cloud);

    bool getIterativeReweighting() const {return this->groupBoxIterativeReweighting->isChecked();}
    int getNReweightingIterations() const {return this->spinBoxNReweightingIterations->value();}

public slots:

    void accept() { done(1); }




};

#endif // Q_PCL_PLUGIN_COMPUTETIMESERIES_HEADER
