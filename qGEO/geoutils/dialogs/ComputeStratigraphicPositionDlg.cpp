#include "ComputeStratigraphicPositionDlg.h"
#include "ui_ComputeStratigraphicPositionDlg.h"


#include <dialogs/FastCloudSelectionDlg.h>

//temp include
#include <iostream>
#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccConsole.h>


ComputeStratigraphicPositionDlg::ComputeStratigraphicPositionDlg(QWidget *parent, BaseFilter *parent_filter) :
    QDialog(parent), m_method(GIVE_NORMAL),
    ui(new Ui::ComputeStratigraphicPositionDlg),
    m_parent_filter(0),
    m_selected_cloud(0)
{
    ui->setupUi(this);

    m_parent_filter = parent_filter;

    //initialize the method names
    m_method_names.push_back(QString("Provide A Normal") );
    m_method_names.push_back(QString ("Estimate from Point set") );

    ui->normal_est_method->addItems(m_method_names);

    connect(this->ui->normal_est_method, SIGNAL(currentIndexChanged(int)), this, SLOT(setMethod(int)));
    connect (this->ui->browse_button, SIGNAL(clicked()), this, SLOT(browse()));
    methodChanged();


}

ComputeStratigraphicPositionDlg::~ComputeStratigraphicPositionDlg()
{
    delete ui;
}

void
ComputeStratigraphicPositionDlg::browse()
{
        FastCloudSelectionDlg * dialog = new FastCloudSelectionDlg;
        ccHObject::Container list;
        m_parent_filter->getAllEntitiesOfType(CC_POINT_CLOUD, list);
        dialog->updateList(list);
        dialog->exec();

        int id = dialog->getSelectedObjectIndex();
        if (id >= 0)
        {
            m_selected_cloud = ccHObjectCaster::ToPointCloud(list.at(id));
            //convert to
            ui->cloud_selected->setText(QString(m_selected_cloud->getName()));
            ui->cloud_selected->setEnabled(true);
        }
        else
            ccConsole::Error("No cloud selected!");


}

void
ComputeStratigraphicPositionDlg::setMethod(int method)
{
    m_method = (method_type) method;
    methodChanged();
}

void
ComputeStratigraphicPositionDlg::methodChanged()
{

    //disable all
    disableAllMethods();

    switch (m_method)
    {
    case (GIVE_NORMAL):
    {
        ui->normal_group->setEnabled(true);
        break;
    }

    case (ESTIMATE_NORMAL_WITH_CLOUD):
    {
        ui->point_set_group->setEnabled(true);
        if (!m_selected_cloud )
        {
            ui->cloud_selected->setEnabled(false);
        }
        else
        {
            ui->cloud_selected->setEnabled(true);
            ui->cloud_selected->setText(QString(m_selected_cloud->getName()));
        }
        break;
    }

    } //end switch
}

void ComputeStratigraphicPositionDlg::disableAllMethods()
{
    ui->normal_group->setEnabled(false);
    ui->point_set_group->setEnabled(false);
}


void ComputeStratigraphicPositionDlg::resetSelections()
{
    m_selected_cloud = 0;
    ui->cloud_selected->setText(QString("None"));
}


