
#include <QToolBar>

#include "PlotterDlg.h"

#include <spc/io/time_series_writer.h>

#include <iostream>


PlotterDlg::PlotterDlg(QWidget* parent): QDialog(parent), Ui::PlotterDlgUi()
{

    setupUi(this);

    //Create a toolbar
    QToolBar *tbar  = new QToolBar(this);
    this->verticalLayout->addWidget(tbar);

    tbar->addAction(this->actionExport);
    tbar->addAction(this->actionSaveTimeSeries);
    tbar->addAction(this->actionClearPlot);

    //must connect any change of range to the props widget

    QCPAxis * ax = this->plot->getMainAxisRect()->getMainAxis();
    connect (ax, SIGNAL(rangeChanged(QCPRange)), this->props, SLOT(setRange(QCPRange)));
    connect (this->props, SIGNAL(rangeChanged(QCPRange)), ax, SLOT(setRange(QCPRange)));

    PropertiesViewerWidget * props = getPropertiesWidget();

    connect(props, SIGNAL(needRedrawing()), getPlotterWidget(), SLOT(replot()));



    connect(actionSaveTimeSeries, SIGNAL(triggered()), this->getPlotterWidget(), SLOT(saveAllSeries()));


}



//QColor
//PlotterDlg::nextColor()
//{
//    if (m_current_color == 18)
//        m_current_color = 6;

//    m_current_color++;
//    return QColor(Qt::GlobalColor(m_current_color)) ;
//}

//void
//PlotterDlg::addCurve(spc::EquallySpacedTimeSeries<float> &tseries)
//{
//    m_series.push_back(tseries);

//    std::vector<float> x = tseries.getX();
//    std::vector<float> y = tseries.getY();
//    this->addCurve(x,y);
//}

//void
//PlotterDlg::addCurve(std::vector<float> &x, std::vector<float> &y)
//{
//    QwtPlotCurve * curve = new QwtPlotCurve;
//    //curve->setRawSamples(&x[0], &y[0], x.size());
//    curve->setRenderHint(QwtPlotItem::RenderAntialiased);
//    curve->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
//    curve->setPen(QPen(nextColor()));

//    QVector<QPointF> * series = new QVector<QPointF>;
//    for (int i = 0; i < x.size(); ++i)
//    {
//        if (std::isnan(x[i]) || std::isnan(y[i]) )
//            continue;

//        QPointF point;
//        point.setX( x[i]);
//        point.setY( y[i]);
//        series->push_back(point);
//    }


//    curve->attach(m_plot);
//    m_curves.push_back(curve);
//    m_vectors.push_back(series);

//    curve->setSamples(*series);

//    double x_min = curve->minXValue();
//    double x_max = curve->maxXValue();

//    double y_min = curve->minYValue();
//    double y_max = curve->maxYValue();

//    // axes
//    m_plot->setAxisTitle(m_plot->xBottom, "x -->" );
//    m_plot->setAxisScale(m_plot->xBottom, x_min, x_max);

//    m_plot->setAxisTitle(m_plot->yLeft, "y -->");
//    m_plot->setAxisScale(m_plot->yLeft, y_min, y_max);

//}




//void
//PlotterDlg::callReplot()
//{
//    m_plot->replot();
//}

void PlotterDlg::addContinousValuesLog(spc::ContinousValuesLog *log)
{
    m_continous_logs.push_back(log); //add to the list
    //a sublayout

//    QCPLayoutGrid *subLayout = new QCPLayoutGrid();
//    plot->plotLayout()->addElement(0, 1, subLayout);


    QCPAxisRect *defaultAxisRect = new QCPAxisRect(plot, false);

    QCPAxis * main = defaultAxisRect->addAxis(QCPAxis::atBottom);
    QCPAxis * other = defaultAxisRect->addAxis(QCPAxis::atLeft);

    defaultAxisRect->setMaximumSize(QSize(100,1000000));

    main->setLabel("values");

    plot->plotLayout()->addElement(0,1,defaultAxisRect);




    QCPGraph * graph = plot->addGraph(other, main);
//    other->setVisible(false);

    connect(other, SIGNAL(rangeChanged(QCPRange)), m_main_axis, SLOT(setRange(QCPRange)));



    std::vector<double> sps = log->getStratigraphicPositions<double>();
    std::vector<double> values = log->getValues<double>();

    std::cout << sps.size() << std::endl;
    std::cout << values.size() << std::endl;

    QVector<double> z = QVector<double>::fromStdVector(sps);
    QVector<double> v = QVector<double>::fromStdVector(values);

//    for (int i = 0; i < z.size(); ++i)
//        std::cout << z.at(i) << std::endl;

//    for (int i = 0; i < v.size(); ++i)
//        std::cout << v.at(i) << std::endl;


//    m_main_axis->setRangeLower(log->getStratigraphicStart());
//    m_main_axis->setRangeUpper(log->getMaxStratigraphicPosition());
    graph->addData(z, v);
    graph->rescaleAxes();
    graph->setPen(QPen(Qt::blue)); // line color blue for first graph
    graph->setLineStyle(QCPGraph::lsLine);
    graph->setBrush(QBrush(QColor(0, 0, 255, 20)));

    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables| QCP::iSelectAxes);

    graph->setVisible(true);
    plot->replot();


}

void PlotterDlg::addRandomContinousValuesLog()
{
    spc::ContinousValuesLog * log = new spc::ContinousValuesLog;
    log->resize(100);

    srand (time(NULL));

    for (int i = 0; i < log->getSize(); ++i)
    {
        int r = rand() % 1000; //range 0-1000

        float n = (r - 500) / 500.0f;

        log->setValue(i, n);
    }

    addContinousValuesLog(log);

}


//void
//PlotterDlg::clearPlot()
//{
//    for ( int item_id = 0; item_id < m_plot->itemList().size(); ++item_id )
//        m_plot->detachItems(item_id);

//    callReplot();
//    m_series.clear();

//    m_current_color = 6;

//}




//void
//PlotterDlg::exportDocument()
//{


//#ifndef QT_NO_PRINTER
//    QString fileName = "plot.pdf";
//#else
//    QString fileName = "plot.png";
//#endif

//#ifndef QT_NO_FILEDIALOG
//    const QList<QByteArray> imageFormats =
//            QImageWriter::supportedImageFormats();

//    QStringList filter;
//    filter += "PDF Documents (*.pdf)";
//#ifndef QWT_NO_SVG
//    filter += "SVG Documents (*.svg)";
//#endif
//    filter += "Postscript Documents (*.ps)";

//    if ( imageFormats.size() > 0 )
//    {
//        QString imageFilter("Images (");
//        for ( int i = 0; i < imageFormats.size(); i++ )
//        {
//            if ( i > 0 )
//                imageFilter += " ";
//            imageFilter += "*.";
//            imageFilter += imageFormats[i];
//        }
//        imageFilter += ")";

//        filter += imageFilter;
//    }

//    fileName = QFileDialog::getSaveFileName(
//                this, "Export File Name", fileName,
//                filter.join(";;"), NULL);
//#endif

//    if ( !fileName.isEmpty() )
//    {
//        QwtPlotRenderer renderer;
//        renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, true);
//        renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true);
//        renderer.setDiscardFlag(QwtPlotRenderer::DiscardCanvasBackground, true);
//        renderer.renderDocument(m_plot, fileName, QSizeF(300, 200), 85);
//    }
//}


//void PlotterDlg::saveCurve()
//{
//    QString filename = "TimeSeries.txt";
//    QStringList filter;
//    filter += "txt (*.txt)";

//    filename = QFileDialog::getSaveFileName(
//                this, "Save Curve", filename,
//                filter.join(";;"), NULL);

//    QFileInfo info = QFileInfo(filename);
//    QString basename = info.baseName();
//    QString path = info.absoluteDir().absolutePath();


//    int n_series = m_series.size();

//    if (!basename.isEmpty() & m_series.size() != 0)
//    {
//        spc::TimeSeriesWriter<float> writer;
//        writer.setASCIIPrecision(6);
//        writer.setSeparator(" ");

//        int count = 0;
//        for (spc::EquallySpacedTimeSeries<float> &series : m_series)
//        {
//            if (n_series == 1)
//                filename = path + QDir::separator() + basename + QString(".txt");
//            else
//                filename = path + QDir::separator() + basename + QString("_%1.txt").arg(count++);


//            spc::GenericTimeSeries<float> * tmp_ptr = &series;

//            writer.setInputSeries(tmp_ptr);
//            writer.setFilename(filename.toStdString());
//            writer.writeAsciiAsSparse();
//        }
//    }

//}



