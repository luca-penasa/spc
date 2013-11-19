#include "ccTimeSeriesGenerator.h"
#include <dialogs/ccTimeSeriesGeneratorEditorDlg.h>


ccTimeSeriesGenerator::ccTimeSeriesGenerator()
{

    QVariant var(QString("Permits to compute time series from cloud, scalar field and model"));
    setMetaData(QString("[qGEO][TimeSeriesGenerator]"), var);
}

void ccTimeSeriesGenerator::initEditDlg()
{        
    m_edit_dlg = new ccTimeSeriesGeneratorEditorDlg(this);
}

void ccTimeSeriesGenerator::updateEditDlg()
{
    ccTimeSeriesGeneratorEditorDlg * asitis = static_cast<ccTimeSeriesGeneratorEditorDlg *> (m_edit_dlg);
    asitis->initWithTree();
}
