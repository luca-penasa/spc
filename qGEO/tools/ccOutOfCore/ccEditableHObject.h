#ifndef CCEDITABLEHOBJECT_H
#define CCEDITABLEHOBJECT_H

#include <QDialog>

class ccEditableHObject
{
public:
    ccEditableHObject();

    bool getHasEditDlg()
    {
        //try to init the dialog, only if it does not exists
        if (!m_edit_dlg)
            initEditDlg();

        //see if the dialog has been init
        if (!m_edit_dlg)
            return false;

        else
            return true;
    }

    virtual void showEditDlg()
    {
        if (getHasEditDlg())
             m_edit_dlg->show();
    }

protected:
    bool m_has_edit_dlg;
    QDialog * m_edit_dlg;

    ///
    /// \brief initEditDlg is the only method you MUST reimplement in subclasses
    ///         if you want to hve a dlg
    ///
    virtual void initEditDlg() {}


};

#endif // CCEDITABLEHOBJECT_H
