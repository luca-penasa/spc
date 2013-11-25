#include "ccTimeSeriesGeneratorEditorDlg.h"
#include "ui_ccTimeSeriesGeneratorEditorDlg.h"

#include <ccOutOfCore/ccTimeSeriesGenerator.h>

#include <qGEO/qGEO.h>



ComputeTimeSeriesDlg::ComputeTimeSeriesDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ccTimeSeriesGeneratorEditorDlgUi)

{
    ui->setupUi(this);
    ui->comboArea->setNone(true); //area may be none!


    connect(this->ui->comboCloud, SIGNAL(currentIndexChanged(int)), this, SLOT(updateScalarFields(int)));
//    connect(qGEO::theInstance(), SIGNAL(selectionChanged(ccHObject::Container&)), this, SLOT(updateWithSelected(ccHObject::Container&)) ) ;

}

ComputeTimeSeriesDlg::~ComputeTimeSeriesDlg()
{
    delete ui;
}

void ComputeTimeSeriesDlg::setInputModels(ccHObject::Container &objects)
{
//    ui->comboModel->clear();
    ui->comboModel->updateObjects(objects);
}

void ComputeTimeSeriesDlg::setInputClouds(ccHObject::Container &objects)
{
//    ui->comboCloud->clear();
    ui->comboCloud->updateObjects(objects);
}

void ComputeTimeSeriesDlg::setInputAreas(ccHObject::Container &objects)
{
//    ui->comboArea->clear();
    ui->comboArea->updateObjects(objects);
}

ccHObject *ComputeTimeSeriesDlg::getSelectedModel() const
{
    return getBackObjectFromCombo(ui->comboModel);

}

ccHObject *ComputeTimeSeriesDlg::getSelectedCloud() const
{
    return getBackObjectFromCombo(ui->comboCloud);
}

ccHObject *ComputeTimeSeriesDlg::getSelectedArea() const
{
    return getBackObjectFromCombo(ui->comboArea);
}

int ComputeTimeSeriesDlg::getSelectedScalarField() const
{
    return this->ui->comboScalar->currentIndex();
}

std::string ComputeTimeSeriesDlg::getSelectedScalarFieldName() const
{
    return this->ui->comboScalar->currentText().toStdString();
}

float ComputeTimeSeriesDlg::getBandwidth() const
{
    return this->ui->spinBandWidth->value();
}

float ComputeTimeSeriesDlg::getStep() const
{
    return this->ui->spinStep->value();
}

ccHObject *ComputeTimeSeriesDlg::getBackObjectFromCombo(const ObjectSelectionComboBox *combo) const
{
    return combo->getSelected();
}


void ComputeTimeSeriesDlg::updateScalarFields(int id)
{
    ccHObject * sel_cloud = getSelectedCloud();
    this->ui->comboScalar->clear();
    if (!sel_cloud)
        return;

    ccPointCloud * cloud = static_cast<ccPointCloud *> (sel_cloud);
    int n = cloud->getNumberOfScalarFields();

    for (int i = 0 ; i < n; ++i)
    {
        std::string name = cloud->getScalarField(i)->getName();
        this->ui->comboScalar->addItem(name.c_str());
    }

}

void ComputeTimeSeriesDlg::updateWithSelected(ccHObject::Container & selected)
{
    ccHObject::Container clouds  = qGEO::theInstance()->getSelectedThatAre(CC_POINT_CLOUD);
    ccHObject::Container models  = qGEO::theInstance()->getSelectedThatHaveMetaData("[qGEO][ccSingleAttitudeModel]");

    std::cout << clouds.size() << std::endl;
    std::cout << models.size() << std::endl;

    setInputClouds(clouds);


    setInputModels(models);




}

void ComputeTimeSeriesDlg::initWithTree()
{
    qGEO * qgeo = qGEO::theInstance(); //get the plutign intself

    ccHObject::Container clouds = qgeo->getAllObjectsInTreeThatAre(CC_POINT_CLOUD);

    std::cout << "found " << clouds.size() << " clouds" << std::endl;
    ccHObject::Container models  = qgeo->getAllObjectsInTreeThatHaveMetaData("[qGEO][ccSingleAttitudeModel]");


    ccHObject::Container selections  = qgeo->getAllObjectsInTreeThatHaveMetaData("[qGEO][ccPlanarSelection]");

    setInputClouds(clouds);
    setInputAreas(selections);
    setInputModels(models);


}



//void AddNewSeriesDlg::updateWithSelected(ccHObject::Container &objects)
//{
//    // clear everything
//    this->ui->comboCloud->clear();
//    this->ui->comboModel->clear();
//    this->ui->comboScalar->clear();


//}
