#ifndef SPC_STRATIGRAPHIC_MODEL_BASE_H
#define SPC_STRATIGRAPHIC_MODEL_BASE_H


#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace spc
{

///!
/// \brief The StratigraphicModelBase class is the base class for all the models representing
/// stratigraphy. It is inherit by all models that permits to evaluate a stratigraphic position
/// and a stratigraphic normal in the 3D space
///!
class StratigraphicModelBase
{  
public:
    typedef typename boost::shared_ptr<StratigraphicModelBase> Ptr;

    //! Def constructor
    StratigraphicModelBase();

    //!
    //! \brief getStratigraphicPosition get the modeled stratigraphic position for a poin in 3d
    //! \param point is the 3d point
    //! \return the stratigraphic position of that point
    //!
    virtual float getStratigraphicPosition(const Eigen::Vector3f point) = 0;

    //!
    //! \brief getStratigraphicNormal get the normal for a given point in 3d
    //! \param point isthe 3d point
    //! \return the normal for that point
    //!
    virtual Eigen::Vector3f getStratigraphicNormal(const Eigen::Vector3f point) = 0;
};

}

#endif // STRATIGRAPHIC_MODEL_BASE_H
