#ifndef SPC_UNDEFORMED_STRATIGRAPHIC_MODEL_H
#define SPC_UNDEFORMED_STRATIGRAPHIC_MODEL_H


#include <pcl/point_types.h>
#include <Eigen/Dense>

//we define here a new point type to be used for storing keypoints and their class-id
struct StratigraphicKeypoint
{
        PCL_ADD_POINT4D;
        int id;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;





template <typename PointT>
class UndeformedStratigraphicModel
{

public:
    /** \brief default constructor
      */
    UndeformedStratigraphicModel(): normal_(Eigen::Vector4f::Zero()), input_changed_(false) { normal_[2] = 1; }

    /** \brief set changed flag to true
      */
    void
    setChangedTrue()
    {
        input_changed_ = true;
    }

    /** \brief set changed flag to false
      */
    void
    setChangedFalse()
    {
        input_changed_ = false;
    }

    /** \brief keypoints changed?
      */
    bool
    hasChanged() const
    {
        return input_changed_;
    }

    /** \brief Evaluate stratigraphic position at given point
      */
    inline float evaluateStratigraphicPosition( Eigen::Vector4f &point) const
    {

        return point.dot(normal_);
    }

private:
     /** \brief vector containing the normal
      */
     Eigen::Vector4f normal_;

     /** \brief is false when input keypoints are changed
       * this will mean the model must be re-optimized before to get any value
       */
     bool input_changed_;
};





#endif // L_UNDEFORMEDSTRATIGRAPHICMODEL_H
