#include "ccSingleAttitudeModelEditorDlg.h"
//#include "ui_ccSingleAttitudeModelEditorDlg.h"
#include <ccOutOfCore/ccSingleAttitudeModel.h>

ccSingleAttitudeModelEditorDlg::ccSingleAttitudeModelEditorDlg(ccSingleAttitudeModel * model, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ccSingleAttitudeModelEditorDlgUi)
{   
    ui->setupUi(this);

    linkToModel(model);
}

ccSingleAttitudeModelEditorDlg::~ccSingleAttitudeModelEditorDlg()
{
    delete ui;
}
