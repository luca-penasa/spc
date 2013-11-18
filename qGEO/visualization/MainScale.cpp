#include "MainScale.h"




MainScale::MainScale(QCustomPlot *parentPlot) : QCPAxisRect(parentPlot, false), m_main_axis(0)
{
    this->setMaximumSize(QSize(200, 100000000)); // we limit the size in width
//    this->setMinimumSize(20, 0);
    m_main_axis = addAxis(QCPAxis::atLeft);
//    m_bottom_axis = addAxis(QCPAxis::atBottom);
//    m_bottom_axis->setVisible(false); // also a bottom


//    QCPLayoutInset * inset = insetLayout();

//    inset->setInsetAlignment(0,Qt::AlignLeft);

    m_main_axis->setLabel("Stratigraphic Position [m]");



    this->setRangeDrag(Qt::Vertical|Qt::Horizontal);





}

QCPAxis *MainScale::getMainAxis() const
{
    return m_main_axis;
}
