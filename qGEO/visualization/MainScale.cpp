#include "MainScale.h"




MainScale::MainScale(QCustomPlot *parentPlot) : QCPAxisRect(parentPlot, false), m_main_axis(0)
{
    this->setMaximumSize(QSize(1000, 1000000000)); // we limit the size in width
    m_main_axis = addAxis(QCPAxis::atLeft);
    m_main_axis->setLabel("Stratigraphic Position [m]");




}

QCPAxis *MainScale::getMainAxis() const
{
    return m_main_axis;
}
