#include "PropertiesViewerWidget.h"
#include "ui_PropertiesViewerWidget.h"

PropertiesViewerWidget::PropertiesViewerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PropertiesViewerWidget)
{
    ui->setupUi(this);

    connect (ui->spinMinSP, SIGNAL(valueChanged(double)), this, SLOT(updatedRange()));
    connect (ui->spinMaxSP, SIGNAL(valueChanged(double)), this, SLOT(updatedRange()));
}

PropertiesViewerWidget::~PropertiesViewerWidget()
{
    delete ui;
}

void PropertiesViewerWidget::setRange(const QCPRange &newRange)
{
    ui->spinMaxSP->setValue(newRange.upper);
    ui->spinMinSP->setValue(newRange.lower);
}

void PropertiesViewerWidget::updatedRange()
{
    emit rangeChanged(QCPRange (ui->spinMinSP->value(), ui->spinMaxSP->value()) );
    emit needRedrawing();

}
