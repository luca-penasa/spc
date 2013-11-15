#ifndef CCSINGLEATTITUDEMODELEDITORDLG_H
#define CCSINGLEATTITUDEMODELEDITORDLG_H

#include <QDialog>
#include <ccOutOfCore/ccSingleAttitudeModel.h>
#include "ui_ccSingleAttitudeModelEditorDlg.h"

//namespace Ui {
//class ccSingleAttitudeModelEditorDlgUi;
//}

class ccSingleAttitudeModelEditorDlg : public QDialog
{
    Q_OBJECT

public:
    //def const
    explicit ccSingleAttitudeModelEditorDlg(ccSingleAttitudeModel * model, QWidget *parent = 0);

    //destructor
    ~ccSingleAttitudeModelEditorDlg();


public slots:
    void stratShiftChanged(double val)
    {
        m_model->setAdditionalShift((float)val);
        m_model->updateInternals();
        m_model->redrawDisplay();
    }

    void minSPChanged(double min_sp)
    {
        m_model->setMinSp(min_sp);
        m_model->updateInternals();
        m_model->redrawDisplay();
        ui->spinMaxSP->setMinimum(min_sp);

    }

    void maxSPChanged(double max_sp)
    {
        m_model->setMaxSp(max_sp);
        m_model->updateInternals();
        m_model->redrawDisplay();
        ui->spinMinSP->setMaximum(max_sp);
    }


private:
    Ui::ccSingleAttitudeModelEditorDlgUi *ui;

protected:

    void initFromModel()
    {
        ui->spinStratShift->setValue((double) m_model->getAdditionalShift());
        ui->spinMaxSP->setValue((double) m_model->getMaxSp());
        ui->spinMinSP->setValue((double) m_model->getMinSp());
    }

    void linkToModel(ccSingleAttitudeModel * model)
    {
        m_model = model;
        initFromModel();
        updateConnections();
    }


    void updateConnections()
    {
        connect (this->ui->spinStratShift, SIGNAL(valueChanged(double)), this, SLOT(stratShiftChanged(double)));
        connect (this->ui->spinMinSP, SIGNAL(valueChanged(double)), this, SLOT(minSPChanged(double)));
        connect (this->ui->spinMaxSP, SIGNAL(valueChanged(double)), this, SLOT(maxSPChanged(double)));

    }

    ccSingleAttitudeModel * m_model;
};

#endif // CCSINGLEATTITUDEMODELEDITORDLG_H
