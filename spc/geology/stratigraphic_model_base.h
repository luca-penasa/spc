#ifndef SPC_STRATIGRAPHIC_MODEL_BASE_H
#define SPC_STRATIGRAPHIC_MODEL_BASE_H


#include <Eigen/Core>
#include <boost/shared_ptr.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace spc
{

using namespace Eigen;

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
    virtual float getStratigraphicPosition(const Vector3f &point) = 0;


    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a whole cloud!
    //! \param cloud
    //! \return a vector of stratigraphic positions computed according this model
    //!
    virtual std::vector<float> getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
       std::vector<float> out;
       for (pcl::PointXYZ p: *cloud)
       {
           pcl::PointXYZ point;
           out.push_back(getStratigraphicPosition(Vector3f(p.x, p.y, p.z)));
       }

       return out;

    }

    //!
    //! \brief getStratigraphicPositions as getStratigraphicPosition but for a whole cloud!
    //! \param cloud
    //! \param indices the indices for which to compute the sp
    //! \return a vector of stratigraphic positions computed according this model
    //!
    virtual std::vector<float> getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices)
    {
       std::vector<float> out;
       for (int id : indices)
       {
           pcl::PointXYZ p = cloud->at(id);
           out.push_back(getStratigraphicPosition(Vector3f(p.x, p.y, p.z)));
       }

       return out;

    }

    //!
    //! \brief getStratigraphicNormal get the normal for a given point in 3d
    //! \param point isthe 3d point
    //! \return the normal for that point
    //!
    virtual Vector3f getStratigraphicNormal(const Vector3f &point) = 0;


    ///! set the parameters for this model
    virtual void setParameters(const VectorXf &parameters)
    {
        pars_ = parameters;
    }


protected:
    VectorXf pars_;

};

}

#endif // STRATIGRAPHIC_MODEL_BASE_H
