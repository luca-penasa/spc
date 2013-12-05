#ifndef PROPERTIESVIEWERWIDGET_H
#define PROPERTIESVIEWERWIDGET_H

#include <QWidget>
#include "qcustomplot.h"

namespace Ui {
class PropertiesViewerWidget;
}

class PropertiesViewerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PropertiesViewerWidget(QWidget *parent = 0);

    ~PropertiesViewerWidget();

public slots:
    void setRange(const QCPRange &newRange);

    void updatedRange();

signals:
     void rangeChanged(const QCPRange &newRange);

     void needRedrawing();

private:
    Ui::PropertiesViewerWidget *ui;
};

#endif // PROPERTIESVIEWERWIDGET_H
