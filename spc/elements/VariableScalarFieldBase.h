#pragma once
#ifndef DYNAMIC_SCALAR_FIELD_GENERATOR_H
#define DYNAMIC_SCALAR_FIELD_GENERATOR_H

#include <spc/core/spc_eigen.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/ElementBase.h>

#include <spc/elements/GeometricElement3DBase.h>
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
class VariableScalarFieldBase : public GeometricElement3DBase
{
public:
    spcTypedefSharedPtrs(VariableScalarFieldBase)
    EXPOSE_TYPE
    //! Def constructor
    VariableScalarFieldBase();

    typedef Eigen::VectorXf VectorT;

    ////////////////////////// MANDATORY METHODS
    //////////////////////////////////////////////////
    //!
	//! \brief getScalarFieldValue( get the modeled stratigraphic position
    //! for a point in 3d
    //! \param point is the 3d point
    //! \return the stratigraphic position of that point
    //!
    virtual float getScalarFieldValue(const Vector3f &point) const = 0;

    //!
	//! \brief getScalarFieldGradient get the normal for a given point in 3d
    //! \param point isthe 3d point
    //! \return the normal for that point
    //!
    virtual Vector3f getScalarFieldGradient(const Vector3f &point) const = 0;


//    //!
//    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a
//    //! whole cloud!
//    //! \param cloud
//    //! \param indices the indices for which to compute the sp
//    //! \return a vector of stratigraphic positions computed according this
//    // model
//    //!
//    virtual VectorT getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
//                                         const std::vector<int> &indices = std::vector<int>()) const;

//    //!
//    //! \brief getStratigraphicPositions with indices
//    //! \param cloud
//    //! \param indices
//    //! \return the stratigraphic positions
//    //!
//    virtual VectorT getScalarFieldValues(PointCloudBase &cloud,
//                                         const std::vector<int> &indices = std::vector<int>()) const;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar,  std::uint32_t const version)
    {
        ar(cereal::base_class<spc::GeometricElement3DBase>(this));
    }
};
}

CEREAL_CLASS_VERSION(spc::VariableScalarFieldBase, 1)

#endif // STRATIGRAPHIC_MODEL_BASE_H
