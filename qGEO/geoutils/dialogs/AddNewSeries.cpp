#include "AddNewSeries.h"
#include "ui_AddNewSeries.h"

AddNewSeries::AddNewSeries(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddNewSeries)
{
    ui->setupUi(this);
}

AddNewSeries::~AddNewSeries()
{
    delete ui;
}

void AddNewSeries::setInputModels(ccHObject::Container &objects)
{
    ui->comboModel->clear();
    ui->comboModel->addObjects(objects);
}

void AddNewSeries::setInputClouds(ccHObject::Container &objects)
{
    ui->comboCloud->clear();
    ui->comboCloud->addObjects(objects);


}

void AddNewSeries::setInputAreas(ccHObject::Container &objects)
{
    ui->comboArea->clear();
    ui->comboArea->addObjects(objects);


}
