#ifndef SPC_STRATIGRAPHIC_MODEL_BASE_H
#define SPC_STRATIGRAPHIC_MODEL_BASE_H


#include <Eigen/Core>
#include <boost/shared_ptr.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spc/elements/generic_cloud.h>


namespace spc
{

using namespace Eigen;

///!
/// \brief The StratigraphicModelBase class is the base class for all the models representing
/// stratigraphy. It is inherit by all models that permits to evaluate a stratigraphic position
/// and a stratigraphic normal in the 3D space
///!
class spcStratigraphicModelBase
{  
public:

    typedef typename boost::shared_ptr<spcStratigraphicModelBase> Ptr;
    typedef typename boost::shared_ptr<const spcStratigraphicModelBase> ConstPtr;

    //! Def constructor
    spcStratigraphicModelBase();


    ////////////////////////// MANDATORY METHODS ///////////////////////////////////////////////
    //!
    //! \brief getStratigraphicPosition get the modeled stratigraphic position for a poin in 3d
    //! \param point is the 3d point
    //! \return the stratigraphic position of that point
    //!
    virtual float getStratigraphicPosition(const Vector3f &point) const = 0;

    //!
    //! \brief getStratigraphicNormal get the normal for a given point in 3d
    //! \param point isthe 3d point
    //! \return the normal for that point
    //!
    virtual Vector3f getStratigraphicNormal(const Vector3f &point) const = 0;



    ///////////////////////// METHODS DEENDING UPON THE MANDATORY ONES ////////////////////////////
    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a whole cloud!
    //! \param cloud
    //! \return a vector of stratigraphic positions computed according this model
    //!
    virtual std::vector<float> getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    //!
    //! \brief getStratigraphicPositions is for the wrapper type of SPC
    //! \param cloud
    //! \return
    //!
    virtual std::vector<float> getStratigraphicPositions(spcGenericCloud * cloud);

    //!
    //! \brief getStratigraphicPositions with indices
    //! \param cloud
    //! \param indices
    //! \return the stratigraphic positions
    //!
    virtual std::vector<float> getStratigraphicPositions(spcGenericCloud *cloud, const std::vector<int> &indices);


    /// also from shared pointer
    virtual std::vector<float> getStratigraphicPositions(spcGenericCloud::Ptr cloud, const std::vector<int> &indices);


    virtual std::vector<float> getStratigraphicPositions(spcGenericCloud::Ptr cloud);

    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a whole cloud!
    //! \param cloud
    //! \param indices the indices for which to compute the sp
    //! \return a vector of stratigraphic positions computed according this model
    //!
    virtual std::vector<float> getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices);




};

}

#endif // STRATIGRAPHIC_MODEL_BASE_H
