#ifndef COMPUTESTRATIGRAPHICPOSITIONDLG_H
#define COMPUTESTRATIGRAPHICPOSITIONDLG_H

#include <QDialog>

#include <ui_ComputeStratigraphicPositionDlg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BaseFilter;
class ccHObject;
class ccPointCloud;


class ComputeStratigraphicPositionDlg : public QDialog, public Ui::ComputeStratigraphicPositionDlg
{
    Q_OBJECT



    //also a string version to be used in combos etc
    QStringList m_method_names;
    
public:

    //methods that can be used for estimating the stratigraphic position
    enum method_type {GIVE_NORMAL=0, ESTIMATE_NORMAL_WITH_CLOUD=1};

    explicit ComputeStratigraphicPositionDlg(QWidget *parent = 0, BaseFilter * parent_filter = 0);
    ~ComputeStratigraphicPositionDlg();

    void resetSelections() ;

    method_type getMethod() {return m_method;}
    ccPointCloud * getSelectedCloud() {return m_selected_cloud;}

    QString getNormalString () {return ui->normal_string->toPlainText();}

    double getIntercept () {return ui->intercept->value();}

    double getSP () {return ui->force_sp->value();}

    bool getCrossSPCheckBox () {return ui->checkBoxNormalToStrata->isChecked();}

private slots:

    void setMethod(int method);

    void browse();


private:
    Ui::ComputeStratigraphicPositionDlg *ui;

    void methodChanged();

    void disableAllMethods();



    method_type m_method;

    BaseFilter * m_parent_filter;

    ccPointCloud * m_selected_cloud;


};

#endif // COMPUTESTRATIGRAPHICPOSITIONDLG_H
