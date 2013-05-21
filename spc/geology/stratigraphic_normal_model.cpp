#include <spc/geology/stratigraphic_normal_model.h>
//#include <ccPlane.h>
//#include <ccHObjectCaster.h>

//TMP
//#include <iostream>

#include<pcl/features/normal_3d.h>


namespace spc
{

template<typename ScalarT>
StratigraphicNormalModel<ScalarT>::StratigraphicNormalModel(): intercept(0)
{
    //default
    normal(0) = 0;
    normal(1) = 0;
    normal(2) = 1; //default is vertical, on z
}

template<typename ScalarT>
int
StratigraphicNormalModel<ScalarT>::setNormalFromSingleCloud(pcl::PointCloud<pcl::PointXYZ> *one_plane_cloud, ScalarT &rms)
{

    Eigen::Vector4f n;
    float c;
    pcl::computePointNormal(*one_plane_cloud, n, c);

    std::cout << n << std::endl;
    normal = n.segment(0,3);

    if(normal(2) < 0.0)
        normal *= -1.0; //always with a positive 'Z' by default!


    normalizeModel();

    return 1;
}

/////INSTANTATIONS

template class StratigraphicNormalModel<float>;
//template class StratigraphicNormalModel<double>;

}//end nspace
