#include "ComputeTimeSeriesDlg.h"

ComputeTimeSeriesDlg::ComputeTimeSeriesDlg(QWidget* parent) : QDialog(parent), Ui::ComputeTimeSeriesDialog()
{
    setupUi(this);


    this->lineEditBasename->setEnabled(false);

}


void ComputeTimeSeriesDlg::updateComboScalars(const ccPointCloud *cloud)
{
    comboScalars->clear();    
    comboScalars->addItemsFromFieldsCloud(cloud);
    
    comboScalars_2->clear();    
    comboScalars_2->addItemsFromFieldsCloud(cloud);


}

