#ifndef ADDNEWSERIES_H
#define ADDNEWSERIES_H

#include <QDialog>
#include <ccHObject.h>
namespace Ui {
class AddNewSeries;
}

class AddNewSeries : public QDialog
{
    Q_OBJECT

public:
    explicit AddNewSeries(QWidget *parent = 0);
    ~AddNewSeries();

    void setInputModels(ccHObject::Container &objects);

    void setInputClouds(ccHObject::Container &objects);

    void setInputAreas(ccHObject::Container &objects);


private:
    Ui::AddNewSeries *ui;


};

#endif // ADDNEWSERIES_H
