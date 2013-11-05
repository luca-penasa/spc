#ifndef EVALUATESTRATIGRAPHICPOSITION_H
#define EVALUATESTRATIGRAPHICPOSITION_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/FastCloudSelectionDlg.h>

#include <spc/geology/single_plane_stratigraphic_model.h>

#include <ccOutOfCore/ccSinglePlaneStratigraphicModel.h>


class EvaluateStratigraphicPosition : public BaseFilter
{
public:    
    EvaluateStratigraphicPosition(ccPluginInterface * parent_plugin = 0);


    virtual int compute() ;

protected:
    virtual int checkSelected();

    int openInputDialog();

private:
    FastCloudSelectionDlg * m_dialog;

    ccSinglePlaneStratigraphicModel * m_model;
};

#endif // EVALUATESTRATIGRAPHICPOSITION_H
