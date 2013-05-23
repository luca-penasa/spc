#include "ComputeTimeSeriesDlg.h"

ComputeTimeSeriesDlg::ComputeTimeSeriesDlg(QWidget* parent) : QDialog(parent), Ui::ComputeTimeSeriesDialog()
{
    setupUi(this);

    this->groupBoxRandomized->setChecked(false);
    this->groupBoxCrossRandomized->setChecked(false);
    this->groupBoxIterativeReweighting->setChecked(false);
}


void ComputeTimeSeriesDlg::updateComboScalars(const ccPointCloud *cloud)
{
    comboScalars->clear();
    comboScalars->addItemsXYZ();
    comboScalars->addItemsFromFieldsCloud(cloud);
    
    comboScalars_2->clear();
    comboScalars_2->addItemsXYZ();
    comboScalars_2->addItemsFromFieldsCloud(cloud);

}

