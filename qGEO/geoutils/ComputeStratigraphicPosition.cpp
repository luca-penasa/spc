#include "ComputeStratigraphicPosition.h"
#include <dialogs/ComputeStratigraphicPositionDlg.h>

//tmp
//#include <iostream>
#include <qtHelper.h>

#include <ccConsole.h>
#include <ccPlane.h>

#include <spc/geology/stratigraphic_normal_model.h>
#include <spc/geology/stratigraphic_evaluator.h>

#include <qPCL/PclUtils/utils/cc2sm.h>

#include <pcl/io/pcd_io.h>


#include <spc/geology/normal_estimator.h>

ComputeStratigraphicPosition::ComputeStratigraphicPosition(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Compute Stratigraphic Position",
                                                                                              "Compute Stratigraphic Position",
                                                                                              "For each point of the cloud compute its stratigraphic position",
                                                                                              ":/toolbar/icons/strat_pos.png")
                                                                                                          , parent_plugin)
{
    m_dialog = 0;
}

int
ComputeStratigraphicPosition::openInputDialog()
{
    if (!m_dialog)
        m_dialog = new ComputeStratigraphicPositionDlg(0, this);

    m_dialog->resetSelections();

    return m_dialog->exec() ? 1 : 0;
}

void
ComputeStratigraphicPosition::getParametersFromDialog()
{
    m_parameters.method =  m_dialog->getMethod();
    m_parameters.normal_string = m_dialog->getNormalString();
    m_parameters.model_intercept = (float) m_dialog->getIntercept();
    m_parameters.sp_value = (float) m_dialog->getSP();
    m_parameters.cloud = m_dialog->getSelectedCloud();

}


int
ComputeStratigraphicPosition::compute()
{

    ccPointCloud * in_cloud = getSelectedEntityAsCCPointCloud();
    if (!in_cloud)
        return 0;


    if (!validateParameters())
    {
        ccConsole::Error("Invalid Parameters!");
        return 0;
    }


    spc::StratigraphicNormalModel model;


    switch(m_parameters.method)
    {
    case (ComputeStratigraphicPositionDlg::GIVE_NORMAL):
    {


        float x,y,z;
        x= m_parameters.normal_vector.x;
        y= m_parameters.normal_vector.y;
        z= m_parameters.normal_vector.z;

        model.setNormal(x,y,z);
        model.setStratigraphicShift(m_parameters.model_intercept);

        ccConsole::Print("Using normal: %f, %f, %f", x,y,z);
        break;

    }

    case (ComputeStratigraphicPositionDlg::ESTIMATE_NORMAL_WITH_CLOUD):
    {

        cc2smReader conv;
        conv.setInputCloud(m_parameters.cloud);
        sensor_msgs::PointCloud2 sm_cloud = conv.getXYZ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(sm_cloud, *pcl_cloud);

        float rms;
        //get an estimate of the normal
        spc::NormalEstimator estimator;
        estimator.addPlanarCloud(pcl_cloud);
        estimator.estimateNormalAsAverage();

        model = estimator.getNormalModel();

        ccConsole::Print("Fitted stratigraphic model with RMS: %f", rms);

        Eigen::Vector3f n = model.getNormal();

        ccConsole::Print("Fitted normal: %f, %f, %f", n(0),n(1),n(2));
        model.setStratigraphicShift(m_parameters.sp_value);
        break;


    }
    } //end sw

    //if normal to strata checkbox is toggled, compute an alternative normal
    if (m_dialog->getCrossSPCheckBox())
    {
        Eigen::Vector3f normal = model.getNormal(); //get back the normal from model
        //we need to compute the cloud's best fit
        ccPlane * plane = in_cloud->fitPlane();
        CCVector3 N(plane->getGLTransformation().getColumn(2));
        Eigen::Vector3f cloud_normal;
        cloud_normal(0) = N[0];
        cloud_normal(1) = N[1];
        cloud_normal(2) = N[2];

        //cross product between the two
        auto new_normal = cloud_normal.cross(normal);

        model.setNormal(new_normal);
        model.setStratigraphicShift(0.0f); //just to be sure!
    }

    ///NOTE we should use here a stratigraphic evaluator class to evaluate the scalar field!
    ccScalarField * sp_field = new ccScalarField;
    sp_field->reserve(in_cloud->size());
    for (int i = 0; i < in_cloud->size(); ++i)
    {
        CCVector3 p;
        in_cloud->getPoint(i, p);

        Eigen::Vector3f eigp (p[0], p[1], p[2]);

        float pos = model.getStratigraphicPosition(eigp);

        sp_field->setValue(i, pos);
    }



    QString def_name = "Stratigraphic Position";

    unsigned trys = 1;
    while (in_cloud->getScalarFieldIndexByName(qPrintable(def_name))>=0 || trys>99)
        def_name = QString("Stratigraphic Position #%1").arg(++trys);

    sp_field->setName(qPrintable(def_name));

    sp_field->computeMinAndMax();

    int index = in_cloud->getScalarFieldIndexByName(sp_field->getName());
    std::cout << index << std::endl;
    if (index >= 0)
    {
        //suggest a new name
        QString new_name = suggestIncrementalName(in_cloud->getScalarFieldName(index) );
        sp_field->setName(new_name.toStdString().c_str());

    }

    int n = in_cloud->addScalarField(sp_field);
    in_cloud->setCurrentDisplayedScalarField(n);
    in_cloud->showSF(true);

    emit entityHasChanged(in_cloud);





    return 1 ;


}

int
ComputeStratigraphicPosition::validateParameters()
{
    switch (m_parameters.method)
    {
    case (ComputeStratigraphicPositionDlg::GIVE_NORMAL):
    {
        if(!ccVector3fromQstring(m_parameters.normal_string, m_parameters.normal_vector))
            return 0;        

        return 1;
    }

    case (ComputeStratigraphicPositionDlg::ESTIMATE_NORMAL_WITH_CLOUD):
    {
        if (!m_parameters.cloud)
            return 0;

        return 1;
    }
    }
}
