#ifndef SPC_STRATIGRAPHIC_NORMAL_MODEL_H
#define SPC_STRATIGRAPHIC_NORMAL_MODEL_H
//#include <ccPointCloud.h>
//#include <ccScalarField.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <spc/geology/stratigraphic_model_base.h>
#include <spc/geology/geologic_plane.h>

namespace spc
{
///
/// \brief The StratigraphicNormalModel class is the same of a GeologicPlane, but
/// with an additional stratigraphic shift associtated to it. This enable you to compute stratigraphic
/// positions, so it inheriths from StratigraphicModelBase. It models stratigraphy with a single
/// normal which is constant for all the 3d space (outcrop). it also have an associated stratigraphic
/// position that is fixed for a given StratigraphicNormalModel.
///
class StratigraphicNormalModel: public GeologicPlane, public StratigraphicModelBase
{
public:
    //! smart pointer type
    typedef typename boost::shared_ptr<StratigraphicNormalModel> Ptr;

    //! constructor
    StratigraphicNormalModel();

    //! set a stratigraphi shift
    /**
     * setStratigraphicShift a point that will lay on the plane rapresented by this model
     * will have this value as stratigraphic position.
     */
    void setStratigraphicShift(float sp) {stratigraphic_shift_ = sp;}

    //! get back model-associated strat pos
    float getStratigraphicShift() const {return stratigraphic_shift_;}

    /// from StratigraphicModelBase
    virtual float getStratigraphicPosition(const Vector3f &point);

    virtual Vector3f getStratigraphicNormal(const Vector3f &point);


    //! get a GeologicPlane from the model
    /**
     * the model is made with a plane in space, from which the stratigraphic position
     * is evaluated as "distance from that plane". This method permits to get other planes
     * rather than the one defining the model itself.
     */

    GeologicPlane::Ptr getGeologicPlaneAtStratigraphicPosition(const float sp, const pcl::PointCloud<pcl::PointXYZ> &cloud ) const;

protected:
    //! the stratigraphic shift for this normal measure
    float stratigraphic_shift_;

};

}//end nspace

#endif
