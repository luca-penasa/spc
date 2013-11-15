#include "FastCloudSelectionDlg.h"
#include "ui_FastCloudSelectionDlg.h"

#include <ccHObject.h>
#include <ccConsole.h>


FastCloudSelectionDlg::FastCloudSelectionDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FastCloudSelectionDlg)
{

    m_selected_object_id = -1;
    ui->setupUi(this);

    connect(this, SIGNAL(accepted()) , this, SLOT(readSelection()) );

}

FastCloudSelectionDlg::~FastCloudSelectionDlg()
{
    delete ui;
}

void
FastCloudSelectionDlg::updateList(ccHObject::Container list)
{    
//    ui->objects_list->reset();

    ui->objects_list->clear();
    for (int i = 0; i < list.size(); ++i)
    {
        ui->objects_list->addItem(list.at(i)->getName());
    }

    m_current_list = list;
}

void
FastCloudSelectionDlg::readSelection()
{
   QModelIndexList selected_items = ui->objects_list->selectionModel()->selectedIndexes();
   if (selected_items.size() != 1)
   {
       return;
   }
   if (selected_items.size() == 0)
   {
       return;
   }

   m_selected_object_id = selected_items.at(0).row();

}



