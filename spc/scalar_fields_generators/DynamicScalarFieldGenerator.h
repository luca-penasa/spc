#pragma once
#ifndef DYNAMIC_SCALAR_FIELD_GENERATOR_H
#define DYNAMIC_SCALAR_FIELD_GENERATOR_H

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
class DynamicScalarFieldGenerator
{  
public:

    typedef  boost::shared_ptr<DynamicScalarFieldGenerator> Ptr;
    typedef  boost::shared_ptr<const DynamicScalarFieldGenerator> ConstPtr;

    //! Def constructor
    DynamicScalarFieldGenerator();


    ////////////////////////// MANDATORY METHODS ///////////////////////////////////////////////
    //!
    //! \brief getStratigraphicPosition get the modeled stratigraphic position for a poin in 3d
    //! \param point is the 3d point
    //! \return the stratigraphic position of that point
    //!
    virtual float getScalarFieldValue(const Vector3f &point) const = 0;

    //!
    //! \brief getStratigraphicNormal get the normal for a given point in 3d
    //! \param point isthe 3d point
    //! \return the normal for that point
    //!
    virtual Vector3f getScalarFieldGradient(const Vector3f &point) const = 0;



    ///////////////////////// METHODS DEPENDING ON THE MANDATORY ONES ////////////////////////////
    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a whole cloud!
    //! \param cloud
    //! \return a vector of stratigraphic positions computed according this model
    //!
    virtual std::vector<float> getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    //!
    //! \brief getStratigraphicPositions is for the wrapper type of SPC
    //! \param cloud
    //! \return
    //!
    virtual std::vector<float> getScalarFieldValues(spcGenericCloud * cloud);

    //!
    //! \brief getStratigraphicPositions with indices
    //! \param cloud
    //! \param indices
    //! \return the stratigraphic positions
    //!
    virtual std::vector<float> getScalarFieldValues(spcGenericCloud *cloud, const std::vector<int> &indices);


    /// also from shared pointer
    virtual std::vector<float> getScalarFieldValues(spcGenericCloud::Ptr cloud, const std::vector<int> &indices);


    virtual std::vector<float> getScalarFieldValues(spcGenericCloud::Ptr cloud);

    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a whole cloud!
    //! \param cloud
    //! \param indices the indices for which to compute the sp
    //! \return a vector of stratigraphic positions computed according this model
    //!
    virtual std::vector<float> getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices);




};

}

#endif // STRATIGRAPHIC_MODEL_BASE_H
