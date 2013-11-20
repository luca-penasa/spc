#ifndef CCEDITABLEHOBJECT_H
#define CCEDITABLEHOBJECT_H

#include <QDialog>
#include <iostream>
#include <qGEO/qGEO.h>

class ccEditableHObject
{
public:
    ccEditableHObject();

    //destructor
    ~ccEditableHObject()
    {
        delete m_edit_dlg;
    }

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
        {
            updateEditDlg(); // do the update
            if (m_edit_dlg_is_freezes_ui)
            {
                    qGEO::theInstance()->getMainAppInterface()->freezeUI(true);
                    m_edit_dlg->show();
            }
            else
                m_edit_dlg->show(); // then show it
        }
    }

    void setEditDlgsFreezesUi(bool status)
    {
        m_edit_dlg_is_freezes_ui = status;
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

    ///
    /// \brief updateEditDlg some objects may need to do somethin before to show the dialog
    /// as re-read the dbtree and update comboBoxes etc...
    ///
    virtual void updateEditDlg()
    {
        // nothing by def
    }

    bool m_edit_dlg_is_freezes_ui;



};

#endif // CCEDITABLEHOBJECT_H
