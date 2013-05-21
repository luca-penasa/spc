//#ifndef SPC_STRATIGRAPHIC_MULTIPLE_PLANE_MODEL_H
//#define SPC_STRATIGRAPHIC_MULTIPLE_PLANE_MODEL_H

////#include <ccPointCloud.h>
////#include <ccScalarField.h>


//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <Eigen/Dense>


//namespace spc
//{

//template <typename ScalarT>
//class StratigraphicMultiplePlanesModel
//{

//    typedef std::vector<ScalarT> ScalarField;
//    typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
//    typedef Eigen::Matrix<ScalarT, 3, 1> Vector3;
//public:
//    StratigraphicMultiplePlanesModel();

//    void addReferenceCloud(CloudXYZ * cloud) {m_reference_clouds.push_back(cloud);}

//    void resetReferenceClouds() {m_reference_clouds.clear();}

//    void setInputCloud(CloudXYZ * cloud){m_cloud = cloud;}

//    auto getAverageSPForReferenceCloud(int id) -> ScalarT;

//    auto getAverageSPForAllClouds() -> ScalarField;

//    auto getSPForReferenceCloud(int id) -> ScalarField;

//    auto getPredictedSPForReferenceCloud(int id) -> ScalarField;

//    auto getPredictedSPForAllReferenceClouds() -> std::vector< ScalarField >;

//    auto getPredictionErrorsForReferenceCloud(int id) -> ScalarField;

//    auto getOverallPredictionErrorsForAllReferenceClouds() -> ScalarField;

//    auto getOverallSumOfSquares() -> ScalarT;


//    std::vector<CloudXYZ *> m_reference_clouds;

//    CloudXYZ * m_cloud;


//    //! the output stratigraphic position scalar field
//    ScalarField * m_field;

//    //! the normal of the model
//    Vector3 * m_normal;

//    //! the intercept of the model
//    ScalarT m_intercept;

//    //! get the stratigraphic position for a point
//    inline ScalarT getSPAt(Vector3 &point)
//    {
//        return point.dot(*m_normal) + m_intercept;
//    }

//    inline Vector3 getSPGradientAt(Vector3 &point)
//    {
//        return m_normal; //is constant, and correspond to the normal
//    }
//};


//} //end nspace
//#endif
