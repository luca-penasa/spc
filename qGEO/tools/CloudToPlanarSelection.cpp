#include "CloudToPlanarSelection.h"

#include <ccGraphicalSegmentationTool.h>

#include <ccPointCloud.h>

#include <qGEO/qGEO.h>

#include <ccGLWindow.h>

#include <cc2DRubberbandLabel.h>

#include <ccPolyline.h>

#include <ccOutOfCore/ccPlanarSelection.h>
#include <iostream>
#include <cc2DViewportLabel.h>




#include<qPCL/PclUtils/utils/cc2sm.h>

CloudToPlanarSelection::CloudToPlanarSelection(ccPluginInterface *parent_plugin): BaseFilter(FilterDescription("From a cloud representing a polygon create a selection",
                                                                     "From a cloud representing a polygon create a selection",
                                                                     "From a cloud representing a polygon create a selection",
                                                                     ":/toolbar/icons/selection.png"), parent_plugin)
{


}

int CloudToPlanarSelection::compute()
{
    return 1;
}

int CloudToPlanarSelection::openInputDialog()
{
    this->setShowProgressBar(false);


    ccHObject::Container cont = qGEO::theInstance()->getSelectedKindOf(CC_POINT_CLOUD);

    if (cont.size() == 0)
        return -1;

    ccGenericPointCloud * verts = dynamic_cast<ccGenericPointCloud *> (cont.at(0));

    if (!verts)
        return -1;

    ccPlanarSelection * selection =  new ccPlanarSelection;

    cc2smReader reader;
    reader.setInputCloud((ccPointCloud *)verts);
    pcl::PointCloud<pcl::PointXYZ> pcl_verts;
    reader.getXYZ(pcl_verts);

    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);


    selection->setVertices(pcl_verts);

    newEntity(selection);


//    sele






//    ccPointCloud * cloud =  this->getSelectedEntityAsCCPointCloud();

//    if(!cloud)
//        return -1;

//    qGEO * interface = static_cast<qGEO *> (this->getParentPlugin());

//    MainWindow * mainw = static_cast<MainWindow *> (interface->getMainAppInterface());

//    ccGLWindow * win = interface->getMainAppInterface()->getActiveGLWindow();

//    ccViewportParameters pars = win->getViewportParameters();

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



//    m_segtool = new ccGraphicalSegmentationTool(mainw);

//    connect(  m_segtool->validButton, SIGNAL( clicked() ), this, SLOT( called() )  );

////    segtool->pauseButton->setEnabled(false);
//    m_segtool->inButton->setEnabled(false);
//    m_segtool->outButton->setEnabled(false);

//    m_segtool->linkWith(win);
//    m_segtool->start();

//    m_segtool->validButton->setEnabled(true);
//    segtool->validAndDeleteButton->setEnabled(true);
//    ccPointCloud * cloud = this->getSelectedEntityAsCCPointCloud();

//    connect(m_segtool, SIGNAL(processFinished(bool)), mainw, SLOT(deactivateSegmentationMode(bool)));

//    mainw->registerMDIDialog(m_segtool,Qt::TopRightCorner);









}

int CloudToPlanarSelection::checkParameters()
{
    ccHObject::Container cont = qGEO::theInstance()->getSelectedKindOf(CC_POINT_CLOUD);

    if (cont.size() == 0)
        return -1;
    else
        return 1;
}

//void CloudToPlanarSelection::called()
//{


//    qGEO * interface = static_cast<qGEO *> (this->getParentPlugin());


//    MainWindow * mainw = static_cast<MainWindow *> (interface->getMainAppInterface());

//    ccGLWindow * win = interface->getMainAppInterface()->getActiveGLWindow();

//    ccViewportParameters pars = win->getViewportParameters();

//    cc2DRubberbandLabel * rubberband = new cc2DRubberbandLabel("Floating Rubberband Selection");
//    rubberband->setParameters(pars);

//    ccPolyline * pline = m_segtool->getPolyLine();

//    ccPointCloud * cloud  = ccPointCloud::From(pline);

//    std::cout << "cloud have " << cloud->size() << " points" << std::endl;



//    rubberband->setVerticesFromCloud(*cloud);
//    rubberband->setVisible(true);

//    newEntity(rubberband);

//    m_segtool->close();
//}

