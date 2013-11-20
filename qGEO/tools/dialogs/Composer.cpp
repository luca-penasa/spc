#include "Composer.h"
#include "ui_Composer.h"

Composer::Composer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Composer)
{
    ui->setupUi(this);
    ui->widget->setInteractions(QCP::iRangeDrag | QCP::iMultiSelect|QCP::iSelectAxes| QCP::iSelectItems);

//    setWindowModality(Qt::ApplicationModal);

}

Composer::~Composer()
{
    delete ui;
}

void Composer::replot()
{
    this->ui->widget->replot();
}
