#ifndef EDIT_H
#define EDIT_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/FastCloudSelectionDlg.h>

#include <spc/stratigraphy/single_attitude_model.h>

#include <ccOutOfCore/ccSingleAttitudeModel.h>


class Edit : public BaseFilter
{
public:    
    Edit(ccPluginInterface * parent_plugin = 0);


    virtual int compute() ;

protected:
    virtual int checkSelected();

//    int openInputDialog();

private:
//    FastCloudSelectionDlg * m_dialog;

//    ccSingleAttitudeModel * m_model;
};

#endif // EDIT_H
