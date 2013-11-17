#ifndef ADDNEWSERIES_H
#define ADDNEWSERIES_H

#include <QDialog>
#include <ccHObject.h>
#include <QComboBox>
#include <qtHelper.h>

namespace Ui {
class AddNewSeries;
}

class AddNewSeriesDlg : public QDialog
{
    Q_OBJECT

public:
    explicit AddNewSeriesDlg(QWidget *parent = 0);
    ~AddNewSeriesDlg();

    void setInputModels(ccHObject::Container &objects);

    void setInputClouds(ccHObject::Container &objects);

    void setInputAreas(ccHObject::Container &objects);

    ccHObject * getSelectedModel() const;

    ccHObject * getSelectedCloud() const;

    ccHObject * getSelectedArea() const;




private:
    Ui::AddNewSeries *ui;

    ccHObject * getBackObjectFromCombo(const ObjectSelectionComboBox *combo) const;

};

#endif // ADDNEWSERIES_H
