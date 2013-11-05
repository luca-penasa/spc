#include "ccProperties.h"
#include <QTreeView>
#include <QVBoxLayout>


#include <QStandardItem>

#include <QStandardItemModel>

#include <iostream> //debug

ccProperties::ccProperties(QWidget *parent) :
    QWidget(parent)
{

    m_tree = new QTreeView;



    QVBoxLayout *mainLayout = new QVBoxLayout;

    mainLayout->addWidget(m_tree);
    setLayout(mainLayout);

    QStandardItem * item = new QStandardItem;

    item->setCheckable(true);

    QStandardItemModel * model = new QStandardItemModel(0,2);
    model->appendRow(item);
    m_tree->setModel(model);

    setWindowTitle(tr("Properties"));


}
