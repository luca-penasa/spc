#include "ComputeStratigraphicPosition.h"
#include <dialogs/ComputeStratigraphicPositionDlg.h>

//tmp
#include <qtHelper.h>

#include <ccConsole.h>
#include <ccPlane.h>

//#include <spc/geology/stratigraphic_normal_model.h>
#include <spc/geology/stratigraphic_evaluator.h>

#include <qPCL/PclUtils/utils/cc2sm.h>

#include <pcl/io/pcd_io.h>

#include <spc/estimators/attitude_estimator.h>


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

//    ccPointCloud * in_cloud = getSelectedEntityAsCCPointCloud();
//    if (!in_cloud)
//        return 0;


//    if (!validateParameters())
//    {
//        ccConsole::Error("Invalid Parameters!");
//        return 0;
//    }


//    spc::SingleAttitudeModel::Ptr model = spc::SingleAttitudeModel::Ptr( new spc::SingleAttitudeModel);

////    spc::SinglePlaneModelFromOneCloudEstimator estimator;


//    switch(m_parameters.method)
//    {
//    case (ComputeStratigraphicPositionDlg::GIVE_NORMAL):
//    {


////        Vector4f pars;
////        pars(3) = m_parameters.model_intercept;

//        float x,y,z;
//        x= m_parameters.normal_vector.x;
//        y= m_parameters.normal_vector.y;
//        z= m_parameters.normal_vector.z;

//        Vector3f pars(x,y,z);


//        model->setNormal(pars);

//        ccConsole::Print("Using normal: %f, %f, %f", pars(0), pars(1), pars(2));
//        break;

//    }

//    case (ComputeStratigraphicPositionDlg::ESTIMATE_NORMAL_WITH_CLOUD):
//    {

//        cc2smReader conv;
//        conv.setInputCloud(m_parameters.cloud);
//        sensor_msgs::PointCloud2 sm_cloud = conv.getXYZ();
//        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::fromROSMsg(sm_cloud, *pcl_cloud);



//        float rms;
//        //get an estimate of the normal
//        spc::AttitudeEstimator estimator;
//        estimator.addInputCloud(pcl_cloud);
//        estimator.estimate();
//        spc::spcAttitude att = estimator.getEstimatedAttitude();

//        Vector3f n = att->getUnitNormal();
//        ccConsole::Print("Fitted normal: %f, %f, %f", n(0),n(1),n(2));

//        break;


//    }
//    } //end sw

//    //if normal to strata checkbox is toggled, compute an alternative normal
//    if (m_dialog->getCrossSPCheckBox())
//    {
//        Eigen::Vector3f normal = model->getUnitNormal(); //get back the normal from model
//        //we need to compute the cloud's best fit
//        ccPlane * plane = ccPlane::Fit( in_cloud );
//        CCVector3 N(plane->getGLTransformation().getColumn(2));
//        Eigen::Vector3f cloud_normal;
//        cloud_normal(0) = N[0];
//        cloud_normal(1) = N[1];
//        cloud_normal(2) = N[2];

//        //cross product between the two
//        auto new_normal = cloud_normal.cross(normal);

//        model->setNormal(new_normal);
//    }

//    ///NOTE we should use here a stratigraphic evaluator class to evaluate the scalar field!
//    ccScalarField * sp_field = new ccScalarField;
//    sp_field->reserve(in_cloud->size());
//    for (int i = 0; i < in_cloud->size(); ++i)
//    {
//        CCVector3 p;
//        in_cloud->getPoint(i, p);

//        Eigen::Vector3f eigp (p[0], p[1], p[2]);

//        float pos = model->getStratigraphicPosition(eigp);

//        sp_field->setValue(i, pos);
//    }



//    QString def_name = "Stratigraphic Position";

//    unsigned trys = 1;
//    while (in_cloud->getScalarFieldIndexByName(qPrintable(def_name))>=0 || trys>99)
//        def_name = QString("Stratigraphic Position #%1").arg(++trys);

//    sp_field->setName(qPrintable(def_name));

//    sp_field->computeMinAndMax();

//    int index = in_cloud->getScalarFieldIndexByName(sp_field->getName());
//    std::cout << index << std::endl;
//    if (index >= 0)
//    {
//        //suggest a new name
//        QString new_name = suggestIncrementalName(in_cloud->getScalarFieldName(index) );
//        sp_field->setName(new_name.toStdString().c_str());

//    }

//    int n = in_cloud->addScalarField(sp_field);
//    in_cloud->setCurrentDisplayedScalarField(n);
//    in_cloud->showSF(true);

//    emit entityHasChanged(in_cloud);





//    return 1 ;


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
