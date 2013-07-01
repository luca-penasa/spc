#ifndef SPC_STRATIGRAPHIC_NORMAL_MODEL_H
#define SPC_STRATIGRAPHIC_NORMAL_MODEL_H
//#include <ccPointCloud.h>
//#include <ccScalarField.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <spc/geology/stratigraphic_model_base.h>

namespace spc
{

template <typename ScalarT>
class StratigraphicNormalModel: public StratigraphicModelBase<ScalarT>
{
public:
    typedef Eigen::Matrix<ScalarT, 3, 1> Vector3;


    //constructor
    StratigraphicNormalModel();

    //! set the stratigraphic normal
    void setNormal(ScalarT x, ScalarT y, ScalarT z) {normal(0) = x; normal(1)=y; normal(2)=z; normalizeModel();}

    //! set normal using a point cloud of points laying on a plane
    int setNormalFromSingleCloud(pcl::PointCloud<pcl::PointXYZ> * one_plane_cloud, ScalarT &rms);

    //! set the intercept of the model
    void setModelIntercept(ScalarT intercept) {intercept = intercept;}

    //////////////////////////////////// GETTERS

    //! get back the normal
    Vector3 getNormal() {return normal;}

    void getNormal(ScalarT &x, ScalarT &y, ScalarT &z) {x = normal(0); y=normal(1); z=normal(2);}

    //! get back model intercept
    ScalarT getModelIntercept() {return intercept;}

private:


    //! the normal of the model
    Vector3  normal;

    //! the intercept of the model
    ScalarT intercept;
public:
    //! get the stratigraphic position for a point
    inline ScalarT getSPAt(const ScalarT &x, const ScalarT &y, const ScalarT &z) const
    {
        Vector3 point;
        point(0) = x; point(1) = y; point(2) = z;
        return point.dot(normal) + intercept;
    }

    inline ScalarT getSPAt(const ScalarT * p) const
    {
        return getSPAt(*p, *(p+1), *(p+2));
    }

    void normalizeModel() {normal = normal/normal.norm();}


};

}//end nspace

#endif
