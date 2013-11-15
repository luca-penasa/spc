#ifndef CCPROPERTIES_H
#define CCPROPERTIES_H

#include <QWidget>
class QTreeView;

class ccProperties : public QWidget
{
    Q_OBJECT
public:
    explicit ccProperties(QWidget *parent = 0);

signals:

public slots:

private:
     QTreeView * m_tree;

};

#endif // CCPROPERTIES_H
