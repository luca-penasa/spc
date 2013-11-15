#ifndef COMPUTESTRATIGRAPHICPOSITION_H
#define COMPUTESTRATIGRAPHICPOSITION_H


#include <qPCL/PclUtils/filters/BaseFilter.h>

#include <dialogs/ComputeStratigraphicPositionDlg.h>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct SPParameters
{
    //constructor
    SPParameters(): method(ComputeStratigraphicPositionDlg::GIVE_NORMAL), normal_string("0 0 1"), normal_vector(0,0,1), model_intercept(0.0), cloud(0), sp_value(0.0)
    {

    }

    ComputeStratigraphicPositionDlg::method_type method;

    //givin a normal
    QString normal_string;
    CCVector3 normal_vector;
    float model_intercept;

    //givin a cloud from which to estimate the normal
    ccPointCloud * cloud;
    float sp_value;

};

class ComputeStratigraphicPosition : public BaseFilter
{

public:

   typedef ComputeStratigraphicPositionDlg::method_type method_type;

    ComputeStratigraphicPosition(ccPluginInterface * parent_plugin = 0);

public:
    int compute() ;

    int openInputDialog();

    void getParametersFromDialog();


private:
    ComputeStratigraphicPositionDlg * m_dialog;
    SPParameters m_parameters;

    int validateParameters();
};

#endif // COMPUTESTRATIGRAPHICPOSITION_H
