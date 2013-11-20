#ifndef COMPOSER_H
#define COMPOSER_H

#include <QWidget>

namespace Ui {
class Composer;
}

class Composer : public QWidget
{
    Q_OBJECT

public:
    explicit Composer(QWidget *parent = 0);
    ~Composer();


    void replot();

private:
    Ui::Composer *ui;
};

#endif // COMPOSER_H
