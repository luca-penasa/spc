#ifndef ADDNEWSERIES_H
#define ADDNEWSERIES_H

#include <QDialog>
#include <ccHObject.h>
#include <QComboBox>
#include <qtHelper.h>

class ccTimeSeriesGenerator;

namespace Ui {
class AddNewSeries;
}

class ccTimeSeriesGeneratorEditorDlg : public QDialog
{
    Q_OBJECT

public:
    explicit ccTimeSeriesGeneratorEditorDlg( ccTimeSeriesGenerator * gen, QWidget *parent = 0);
    ~ccTimeSeriesGeneratorEditorDlg();

    void setInputModels(ccHObject::Container &objects);

    void setInputClouds(ccHObject::Container &objects);

    void setInputAreas(ccHObject::Container &objects);

    ccHObject * getSelectedModel() const;

    ccHObject * getSelectedCloud() const;

    ccHObject * getSelectedArea() const;

    int getSelectedScalarField() const;

    std::string getSelectedScalarFieldName() const;

    float getBandwidth() const;

    float getStep() const;

public slots:

    void updateScalarFields(int id) ;

//    void updateWithSelected(ccHObject::Container & objects);



private:
    Ui::AddNewSeries *ui;

    ccTimeSeriesGenerator * m_generator;

    ccHObject * getBackObjectFromCombo(const ObjectSelectionComboBox *combo) const;

};

#endif // ADDNEWSERIES_H
