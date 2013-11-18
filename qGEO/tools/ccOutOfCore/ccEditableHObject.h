#ifndef CCEDITABLEHOBJECT_H
#define CCEDITABLEHOBJECT_H

#include <QDialog>
#include <iostream>

class ccEditableHObject
{
public:
    ccEditableHObject();

    bool getHasEditDlg()
    {
        //try to init the dialog, only if it does not exists
        if (!m_edit_dlg)
        {
            std::cout << "trying to intiialize dialog, it does not exists yet" << std::endl;
            initEditDlg();
        }

        //see if the dialog has been init
        if (!m_edit_dlg)
        {
            std::cout << "the dialog still do not exist, have you implemented the initEditDlg method?" <<std::endl;
            return false;
        }

        else
            return true;
    }

    virtual void showEditDlg()
    {
        if (getHasEditDlg())
             m_edit_dlg->show();
    }

protected:
    QDialog * m_edit_dlg;

    ///
    /// \brief initEditDlg is the only method you MUST reimplement in subclasses
    ///         if you want to hve a dlg
    ///
    virtual void initEditDlg()
    {
        std::cout << "ccEditableHObject initEditDlg called... are you sure?" << std::endl;
    }


};

#endif // CCEDITABLEHOBJECT_H
