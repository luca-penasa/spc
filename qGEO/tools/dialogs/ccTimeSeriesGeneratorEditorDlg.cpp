#include "ccTimeSeriesGeneratorEditorDlg.h"
#include "ui_AddNewSeries.h"

#include <ccOutOfCore/ccTimeSeriesGenerator.h>

ccTimeSeriesGeneratorEditorDlg::ccTimeSeriesGeneratorEditorDlg(ccTimeSeriesGenerator *gen, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddNewSeries),
    m_generator(0)

{
    ui->setupUi(this);
    ui->comboArea->setNone(true); //area may be none!

    m_generator = gen;

    connect(this->ui->comboCloud, SIGNAL(currentIndexChanged(int)), this, SLOT(updateScalarFields(int)));
}

ccTimeSeriesGeneratorEditorDlg::~ccTimeSeriesGeneratorEditorDlg()
{
    delete ui;
}

void ccTimeSeriesGeneratorEditorDlg::setInputModels(ccHObject::Container &objects)
{
    ui->comboModel->clear();
    ui->comboModel->addObjects(objects);
}

void ccTimeSeriesGeneratorEditorDlg::setInputClouds(ccHObject::Container &objects)
{
    ui->comboCloud->clear();
    ui->comboCloud->addObjects(objects);
}

void ccTimeSeriesGeneratorEditorDlg::setInputAreas(ccHObject::Container &objects)
{
    ui->comboArea->clear();
    ui->comboArea->addObjects(objects);
}

ccHObject *ccTimeSeriesGeneratorEditorDlg::getSelectedModel() const
{
    return getBackObjectFromCombo(ui->comboModel);

}

ccHObject *ccTimeSeriesGeneratorEditorDlg::getSelectedCloud() const
{
    return getBackObjectFromCombo(ui->comboCloud);
}

ccHObject *ccTimeSeriesGeneratorEditorDlg::getSelectedArea() const
{
    return getBackObjectFromCombo(ui->comboArea);
}

int ccTimeSeriesGeneratorEditorDlg::getSelectedScalarField() const
{
    return this->ui->comboScalar->currentIndex();
}

std::string ccTimeSeriesGeneratorEditorDlg::getSelectedScalarFieldName() const
{
    return this->ui->comboScalar->currentText().toStdString();
}

float ccTimeSeriesGeneratorEditorDlg::getBandwidth() const
{
    return this->ui->spinBandWidth->value();
}

float ccTimeSeriesGeneratorEditorDlg::getStep() const
{
    return this->ui->spinStep->value();
}

ccHObject *ccTimeSeriesGeneratorEditorDlg::getBackObjectFromCombo(const ObjectSelectionComboBox *combo) const
{

    return combo->getSelected();
}


void ccTimeSeriesGeneratorEditorDlg::updateScalarFields(int id)
{
    ccHObject * sel_cloud = getSelectedCloud();


    this->ui->comboScalar->clear();

    ccPointCloud * cloud = static_cast<ccPointCloud *> (sel_cloud);
    int n = cloud->getNumberOfScalarFields();

    for (int i = 0 ; i < n; ++i)
    {
        std::string name = cloud->getScalarField(i)->getName();
        this->ui->comboScalar->addItem(name.c_str());
    }

}

//void AddNewSeriesDlg::updateWithSelected(ccHObject::Container &objects)
//{
//    // clear everything
//    this->ui->comboCloud->clear();
//    this->ui->comboModel->clear();
//    this->ui->comboScalar->clear();


//}
