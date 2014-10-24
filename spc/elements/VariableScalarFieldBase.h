#pragma once
#ifndef DYNAMIC_SCALAR_FIELD_GENERATOR_H
#define DYNAMIC_SCALAR_FIELD_GENERATOR_H

#include <spc/methods/spc_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/ElementBase.h>
namespace spc
{

using namespace Eigen;

///!
/// \brief The StratigraphicModelBase class is the base class for all the models
/// representing
/// stratigraphy. It is inherit by all models that permits to evaluate a
/// stratigraphic position
/// and a stratigraphic normal in the 3D space
///!
class VariableScalarFieldBase : public ElementBase
{
public:
    SPC_OBJECT(VariableScalarFieldBase)
    EXPOSE_TYPE
    //! Def constructor
    VariableScalarFieldBase();

    ////////////////////////// MANDATORY METHODS
    //////////////////////////////////////////////////
    //!
    //! \brief getStratigraphicPosition get the modeled stratigraphic position
    // for a poin in 3d
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

    ///////////////////////// METHODS DEPENDING ON THE MANDATORY ONES
    ///////////////////////////////
    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a
    // whole cloud!
    //! \param cloud
    //! \return a vector of stratigraphic positions computed according this
    // model
    //!
    virtual std::vector<float>
    getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

    //!
    //! \brief getStratigraphicPositions is for the wrapper type of SPC
    //! \param cloud
    //! \return
    //!
    virtual std::vector<float> getScalarFieldValues(PointCloudBase
                                                    *cloud) const;

    //!
    //! \brief getStratigraphicPositions with indices
    //! \param cloud
    //! \param indices
    //! \return the stratigraphic positions
    //!
    virtual std::vector<float> getScalarFieldValues(PointCloudBase *cloud,
                                                    const std::vector
                                                    <int> &indices) const;

    /// also from shared pointer
    virtual std::vector<float> getScalarFieldValues(PointCloudBase::Ptr cloud,
                                                    const std::vector
                                                    <int> &indices) const;

    virtual std::vector<float>
    getScalarFieldValues(PointCloudBase::Ptr cloud) const;

    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a
    // whole cloud!
    //! \param cloud
    //! \param indices the indices for which to compute the sp
    //! \return a vector of stratigraphic positions computed according this
    // model
    //!
    virtual std::vector<float> getScalarFieldValues(pcl::PointCloud
                                                    <pcl::PointXYZ>::Ptr cloud,
                                                    const std::vector
                                                    <int> &indices) const;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this));
    }
};
}

#endif // STRATIGRAPHIC_MODEL_BASE_H
