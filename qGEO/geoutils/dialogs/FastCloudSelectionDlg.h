#ifndef FASTCLOUDSELECTIONDLG_H
#define FASTCLOUDSELECTIONDLG_H

#include <QDialog>

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

    void updateList(std::vector<ccHObject *> list);

    int getSelectedObjectIndex() {return m_selected_object;}

private slots:
    void readSelection();
    
private:
    Ui::FastCloudSelectionDlg *ui;
    int m_selected_object;
};

#endif // FASTCLOUDSELECTIONDLG_H
