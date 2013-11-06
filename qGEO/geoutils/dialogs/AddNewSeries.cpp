#include "AddNewSeries.h"
#include "ui_AddNewSeries.h"

AddNewSeriesDlg::AddNewSeriesDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddNewSeries)
{
    ui->setupUi(this);
    ui->comboArea->setNone(true); //area may be none!

}

AddNewSeriesDlg::~AddNewSeriesDlg()
{
    delete ui;
}

void AddNewSeriesDlg::setInputModels(ccHObject::Container &objects)
{
    ui->comboModel->clear();
    ui->comboModel->addObjects(objects);
}

void AddNewSeriesDlg::setInputClouds(ccHObject::Container &objects)
{
    ui->comboCloud->clear();
    ui->comboCloud->addObjects(objects);
}

void AddNewSeriesDlg::setInputAreas(ccHObject::Container &objects)
{
    ui->comboArea->clear();
    ui->comboArea->addObjects(objects);
}

ccHObject *AddNewSeriesDlg::getSelectedModel() const
{
    return getBackObjectFromCombo(ui->comboModel);

}

ccHObject *AddNewSeriesDlg::getSelectedCloud() const
{
    return getBackObjectFromCombo(ui->comboCloud);
}

ccHObject *AddNewSeriesDlg::getSelectedArea() const
{
    return getBackObjectFromCombo(ui->comboArea);
}

ccHObject *AddNewSeriesDlg::getBackObjectFromCombo(const QComboBox *combo) const
{

    QVariant data = combo->itemData(combo->currentIndex());

    ccHObject * object = static_cast<ccHObject *> (data.data());

    return object;
}
