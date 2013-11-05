#ifndef FASTCLOUDSELECTIONDLG_H
#define FASTCLOUDSELECTIONDLG_H

#include <QDialog>

#include <ccHObject.h>

class ccHObject;

namespace Ui {
class FastCloudSelectionDlg;
}

class FastCloudSelectionDlg : public QDialog
{
    Q_OBJECT
    
public:
    explicit FastCloudSelectionDlg(QWidget *parent = 0);
    ~FastCloudSelectionDlg();

    void updateList(ccHObject::Container list);

    int getSelectedObjectIndex() {return m_selected_object_id;}

    ccHObject * getSelectedObject() const
    {
        return m_current_list.at(m_selected_object_id);
    }

private slots:
    void readSelection();
    
private:
    Ui::FastCloudSelectionDlg *ui;
    int m_selected_object_id;

    ccHObject::Container m_current_list;
};

#endif // FASTCLOUDSELECTIONDLG_H
