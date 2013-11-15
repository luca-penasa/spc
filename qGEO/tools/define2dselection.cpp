#include "define2dselection.h"

#include <ccGraphicalSegmentationTool.h>

#include <ccPointCloud.h>

#include <qGEO/qGEO.h>

#include <ccGLWindow.h>

#include <cc2DRubberbandLabel.h>

#include <ccPolyline.h>


#include <cc2DViewportLabel.h>
Define2DSelection::Define2DSelection(ccPluginInterface *parent_plugin): BaseFilter(FilterDescription("Open Properties Dialog",
                                                                     "do something",
                                                                     "Def selection",
                                                                     ":/toolbar/icons/test1.png"), parent_plugin)
{


}

int Define2DSelection::compute()
{








    return 1;
}

int Define2DSelection::openInputDialog()
{
    ccPointCloud * cloud =  this->getSelectedEntityAsCCPointCloud();

    if(!cloud)
        return -1;

    qGEO * interface = static_cast<qGEO *> (this->getParentPlugin());

    MainWindow * mainw = static_cast<MainWindow *> (interface->getMainAppInterface());

    ccGLWindow * win = interface->getMainAppInterface()->getActiveGLWindow();

    ccViewportParameters pars = win->getViewportParameters();

//    cc2DViewportLabel * label = new cc2DViewportLabel("test labels");

//    float roi[] = {0, 0, 100, 80};

//    label->setRoi(roi);
//    label->setParameters(pars);

//    label->setVisible(true);

//    newEntity(label);




//    qGEO * interface = static_cast<qGEO *> (this->getParentPlugin());


//    MainWindow * mainw = static_cast<MainWindow *> (interface->getMainAppInterface());

//    ccGLWindow * win = interface->getMainAppInterface()->getActiveGLWindow();

//    ccViewportParameters pars = win->getViewportParameters();



    m_segtool = new ccGraphicalSegmentationTool(mainw);

    connect(  m_segtool->validButton, SIGNAL( clicked() ), this, SLOT( called() )  );

//    segtool->pauseButton->setEnabled(false);
    m_segtool->inButton->setEnabled(false);
    m_segtool->outButton->setEnabled(false);

    m_segtool->linkWith(win);
    m_segtool->start();

    m_segtool->validButton->setEnabled(true);
//    segtool->validAndDeleteButton->setEnabled(true);
//    ccPointCloud * cloud = this->getSelectedEntityAsCCPointCloud();

//    connect(m_segtool, SIGNAL(processFinished(bool)), mainw, SLOT(deactivateSegmentationMode(bool)));

//    mainw->registerMDIDialog(m_segtool,Qt::TopRightCorner);









}

void Define2DSelection::called()
{


    qGEO * interface = static_cast<qGEO *> (this->getParentPlugin());


    MainWindow * mainw = static_cast<MainWindow *> (interface->getMainAppInterface());

    ccGLWindow * win = interface->getMainAppInterface()->getActiveGLWindow();

    ccViewportParameters pars = win->getViewportParameters();

    cc2DRubberbandLabel * rubberband = new cc2DRubberbandLabel("Floating Rubberband Selection");
    rubberband->setParameters(pars);

    ccPolyline * pline = m_segtool->getPolyLine();

    ccPointCloud * cloud  = ccPointCloud::From(pline);

    std::cout << "cloud have " << cloud->size() << " points" << std::endl;



    rubberband->setVerticesFromCloud(*cloud);
    rubberband->setVisible(true);

    newEntity(rubberband);

    m_segtool->close();
}

