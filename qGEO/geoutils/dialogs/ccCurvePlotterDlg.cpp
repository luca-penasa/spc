
//#include "plotter.h"

#include <cmath>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_legend.h>
#include <qwt_text.h>
#include <QVector>

#include <qwt_math.h>
#include <qwt_scale_engine.h>
#include <qwt_symbol.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_text.h>

#include <qwt_plot_canvas.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_magnifier.h>
#include <qwt_series_data.h>
#include <iostream>
#include <QImageWriter>

#include <qwt_plot_renderer.h>
#include<QFileDialog>

#include <fstream>
#include <QToolBar>

#include "ccCurvePlotterDlg.h"


ccCurvePlotterDlg::ccCurvePlotterDlg(QWidget* parent): QDialog(parent), Ui::CurvePlotterDialog()
{

    setupUi(this);

    //Create a toolbar
    QToolBar *tbar  = new QToolBar(this);
    this->verticalLayout->addWidget(tbar);

    tbar->addAction(this->actionExport);
    tbar->addAction(this->actionSaveTimeSeries);
    tbar->addAction(this->actionClearPlot);


    m_plot = this->qwtPlot;

    m_plot->setAutoReplot(true);
    // panning with the left mouse button
    (void) new QwtPlotPanner( m_plot->canvas() );

    // zoom in/out with the wheel
    (void) new QwtPlotMagnifier( m_plot->canvas() );

    setWindowFlags( Qt::Dialog & Qt::WindowMinMaxButtonsHint);



    connect(this->actionExport, SIGNAL(triggered()), SLOT(exportDocument()));
    connect(this->actionSaveTimeSeries, SIGNAL(triggered()), SLOT(saveCurve()));
    connect(this->actionClearPlot, SIGNAL(triggered()), SLOT(clearPlot()));

    m_current_color = 6;
};

QColor
ccCurvePlotterDlg::nextColor()
{
    if (m_current_color == 18)
        m_current_color = 6;

    m_current_color++;
    return QColor(Qt::GlobalColor(m_current_color)) ;
}

template <typename sType>
void
ccCurvePlotterDlg::addCurve(spc::GenericTimeSeries<sType> * tseries)
{
    auto x = tseries->getX();
    auto y = tseries->getY();
    this->addCurve<sType>(x,y);
}

template <class sType>
void
ccCurvePlotterDlg::addCurve(std::vector<sType> &x, std::vector<sType> &y)
{
    QwtPlotCurve * curve = new QwtPlotCurve;
    //curve->setRawSamples(&x[0], &y[0], x.size());
    curve->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
    curve->setPen(QPen(nextColor()));

    QVector<QPointF> * series = new QVector<QPointF>;
    for (int i = 0; i < x.size(); ++i)
    {
        if (std::isnan(x[i]) || std::isnan(y[i]) )
            continue;

        QPointF point;
        point.setX( (sType) x[i]);
        point.setY( (sType) y[i]);
        series->push_back(point);
    }

    curve->attach(m_plot);
    m_curves.push_back(curve);
    m_vectors.push_back(series);

    curve->setSamples(*series);

    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());

    double y_min = *std::min_element(y.begin(), y.end());
    double y_max = *std::max_element(y.begin(), y.end());

    // axes
    m_plot->setAxisTitle(m_plot->xBottom, "x -->" );
    m_plot->setAxisScale(m_plot->xBottom, x_min, x_max);

    m_plot->setAxisTitle(m_plot->yLeft, "y -->");
    m_plot->setAxisScale(m_plot->yLeft, y_min, y_max);

}




void
ccCurvePlotterDlg::callReplot()
{
    m_plot->replot();
}


void
ccCurvePlotterDlg::clearPlot()
{
    for ( int item_id = 0; item_id < m_plot->itemList().size(); ++item_id )
        m_plot->detachItems(item_id);

    callReplot();

    m_current_color = 6;

}




void
ccCurvePlotterDlg::exportDocument()
{
#ifndef QT_NO_PRINTER
    QString fileName = "plot.pdf";
#else
    QString fileName = "plot.png";
#endif

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
            QImageWriter::supportedImageFormats();

    QStringList filter;
    filter += "PDF Documents (*.pdf)";
#ifndef QWT_NO_SVG
    filter += "SVG Documents (*.svg)";
#endif
    filter += "Postscript Documents (*.ps)";

    if ( imageFormats.size() > 0 )
    {
        QString imageFilter("Images (");
        for ( int i = 0; i < imageFormats.size(); i++ )
        {
            if ( i > 0 )
                imageFilter += " ";
            imageFilter += "*.";
            imageFilter += imageFormats[i];
        }
        imageFilter += ")";

        filter += imageFilter;
    }

    fileName = QFileDialog::getSaveFileName(
                this, "Export File Name", fileName,
                filter.join(";;"), NULL);
#endif

    if ( !fileName.isEmpty() )
    {
        QwtPlotRenderer renderer;
        renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, true);
        renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true);
        renderer.setDiscardFlag(QwtPlotRenderer::DiscardCanvasBackground, true);
        renderer.renderDocument(m_plot, fileName, QSizeF(300, 200), 85);
    }
}


void ccCurvePlotterDlg::saveCurve()
{
    QString filename = "TimeSeries.txt";
    QStringList filter;
    filter += "txt (*.txt)";

    filename = QFileDialog::getSaveFileName(
                this, "Save Curve", filename,
                filter.join(";;"), NULL);

    std::ofstream file;

    if (!filename.isEmpty() & m_vectors.size() != 0)
    {
        //for now only the last one
        QVector<QPointF> * vector = m_vectors[m_vectors.size()-1];
        file.open (filename.toStdString().c_str());
        for (size_t i = 0; i < vector->size(); ++i)
        {
            QPointF point = vector->at(i);
            file << point.x() << " " << point.y() << "\n";
        }
        file.close();
    }

}

//instantiatinos!!
template
void
ccCurvePlotterDlg::addCurve<float>(std::vector< float >&x, std::vector< float > &y);

template
void
ccCurvePlotterDlg::addCurve<double>(std::vector< double >&x, std::vector< double > &y);

template
void
ccCurvePlotterDlg::addCurve<float>(spc::GenericTimeSeries<float> * tseries);

template
void
ccCurvePlotterDlg::addCurve<double>(spc::GenericTimeSeries<double> * tseries);
