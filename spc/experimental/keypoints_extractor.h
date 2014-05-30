#ifndef KEYPOINTS_EXTRACTOR_H
#define KEYPOINTS_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spc/common/point_types.h>

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>

//#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/keypoints/impl/uniform_sampling.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/filters/voxel_grid.h>

#include <spc/methods/compute_eigen_indices.h>
#include <fstream>

#include <pcl/common/centroid.h>
namespace spc
{

template <typename PointInT> class Keypoint
{
public:
    typedef typename boost::shared_ptr<Keypoint<PointInT>> Ptr;
    typedef typename pcl::PointCloud<PointInT>::Ptr CloudPtrT;
    typedef PointInT PointT;

    Keypoint()
    {
    } // def const

    PointInT getPoint(const int id)
    {
        return cloud_->at(id);
    }

    CloudPtrT cloud_; // a pointer to the cloud it refers to
    int id_;
    float b_distance_from_center_;
    float bw_ratio_;
    int total_n_points_;
    float avg_int_black_, avg_int_white_;
    float std_int_black_, std_int_white_;
    float lambda1, lambda2;

    Eigen::Vector3f v;
};

template <typename KeypointT>
class Keypoints : public std::vector<typename KeypointT::Ptr>
{
public:
    typedef typename boost::shared_ptr<Keypoints<KeypointT>> Ptr;

    Keypoints()
    {
    } // const

    void saveToFile(std::string filename)
    {
        std::ofstream stream;
        stream.open(filename.c_str());

        float x, y, z, intensity;
        for (int i = 0; i < this->size(); ++i) {
            typename KeypointT::Ptr key = this->at(i);
            typename KeypointT::PointT p = key->getPoint(i);

            x = p.x;
            y = p.y;
            z = p.z;
            intensity = p.intensity;

            std::stringstream string_stream;
            string_stream << x << " " << y << " " << z << " " << intensity
                          << " " << key->b_distance_from_center_ << " "
                          << key->bw_ratio_ << " " << key->total_n_points_
                          << " " << key->avg_int_black_ << " "
                          << key->std_int_black_ << " " << key->avg_int_white_
                          << " " << key->std_int_white_ << " " << key->lambda1
                          << " " << key->lambda2 << " " << key->v(0) << " "
                          << key->v(1) << " " << key->v(2) << " "
                          << "\n";
            stream << string_stream.str().c_str();
        }
        stream.close();
    }
};

template <typename PointInT> class KeypointsExtractor
{
public:
    typedef typename pcl::PointCloud<PointInT> CloudT;
    typedef typename CloudT::Ptr CloudPtrT;

    typedef typename pcl::search::FlannSearch<PointInT> FlannSearchT;
    typedef typename FlannSearchT::Ptr FlannSearchPtrT;

    typedef Keypoint<PointInT> KeypointT;
    typedef typename Keypoint<PointInT>::Ptr KeypointPtrT;

    typedef Keypoints<KeypointT> KeypointsT;
    typedef typename Keypoints<KeypointT>::Ptr KeypointsPtrT;

    //    typedef typename pcl::UniformSampling<PointInT> UniformSamplingT;

public:
    KeypointsExtractor();

    void setInCloud(CloudPtrT in_cloud)
    {
        in_cloud_ = in_cloud;
    }

    CloudPtrT getDownsampledVersion(float leaf_size)
    {

        if (leaf_size == 0.0) {
            downsampled_versions_.insert(
                std::pair<float, CloudPtrT>(leaf_size, in_cloud_));
            return in_cloud_;
        }

        if (downsampled_versions_.find(leaf_size)
            != downsampled_versions_.end()) {
            CloudPtrT out_cloud = downsampled_versions_.at(leaf_size);
            return out_cloud;

        } else // compute it!
        {
            CloudPtrT out_cloud = CloudPtrT(new CloudT);

            pcl::VoxelGrid<PointInT> uniform_sampler;
            uniform_sampler.setInputCloud(in_cloud_);
            uniform_sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
            uniform_sampler.filter(*out_cloud);

            downsampled_versions_.insert(
                std::pair<float, CloudPtrT>(leaf_size, out_cloud));

            return out_cloud;
        }
    }

    KeypointsPtrT computeKeypoints(float radius, float leafsize)
    {

        KeypointsPtrT keys(new KeypointsT);

        CloudPtrT cloud = getDownsampledVersion(leafsize);
        FlannSearchPtrT searcher = getFlannSearcher(cloud);

        //#pragma omp parallel for
        for (int i = 0; i < cloud->size(); ++i) {

            //            std::cout << "computing for index " << i << std::endl;
            KeypointPtrT key(new KeypointT);
            PointInT point = cloud->at(i);
            std::vector<int> neighbors_ids
                = getNeighborsInRadius(leafsize, radius, point);
            key->cloud_ = cloud;
            key->total_n_points_ = neighbors_ids.size();

            float ratio;
            std::vector<int> white, blacks;
            blacks = thresholdCloud(cloud, neighbors_ids, ratio, white);
            key->bw_ratio_ = ratio;

            float distance = getDistanceFromCenterOfMass(point, cloud, blacks);
            key->b_distance_from_center_ = distance;

            float b_avg = computeIntensityMean(cloud, blacks);
            key->avg_int_black_ = b_avg;
            float w_avg = computeIntensityMean(cloud, white);
            key->avg_int_white_ = w_avg;

            float b_std = computeIntensityStandardDeviation(cloud, blacks);
            key->std_int_black_ = b_std;
            float w_std = computeIntensityStandardDeviation(cloud, white);
            key->std_int_white_ = w_std;

            float lam1, lam2;
            Eigen::Vector3f v;
            fitLine(cloud, blacks, lam1, lam2, v);

            key->lambda1 = lam1;
            key->lambda2 = lam2;

            key->v = v;

            keys->push_back(key);
        }

        return keys;
    }

    FlannSearchPtrT getFlannSearcher(CloudPtrT cloud)
    {
        if (searchers_.find(cloud) != searchers_.end()) {
            return searchers_.at(cloud);
        } else {
            FlannSearchPtrT searcher = FlannSearchPtrT(new FlannSearchT);
            searcher->setInputCloud(cloud);
            searchers_.insert(std::pair
                              <CloudPtrT, FlannSearchPtrT>(cloud, searcher));

            return searcher;
        }
    }

    std::vector<int> getNeighborsInRadius(float scale, float radius,
                                          typename PointInT point)
    {
        CloudPtrT cloud = getDownsampledVersion(scale);
        FlannSearchPtrT flann = getFlannSearcher(cloud);

        std::vector<int> indices;
        std::vector<float> distances;

        flann->radiusSearch(point, radius, indices, distances);

        return indices;
    }

    Eigen::Vector4f getFittingPlane(CloudPtrT cloud)
    {
        Eigen::Vector4f normal;
        float c;
        pcl::computePointNormal(*cloud, normal, c);

        return normal;
    }

    std::vector<int> thresholdCloud(CloudPtrT in_cloud,
                                    std::vector<int> indices, float &ratio,
                                    std::vector<int> &others)
    {
        std::vector<int> out_indices;

        // compute the average
        float avg = computeIntensityMean(in_cloud, indices);

        float val;

        for (int i = 0; i < indices.size(); ++i) {
            val = in_cloud->at(i).intensity;
            if (val <= avg)
                out_indices.push_back(i);
            else
                others.push_back(i);
        }

        ratio = (float)out_indices.size() / (float)others.size();
        return out_indices;
    }

    void solveEigenvectorEigenvalues(const Eigen::Matrix3f matrix,
                                     Eigen::Matrix3f &eigenvectors,
                                     Eigen::Vector3f &eigenvalues)
    {

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(matrix);

        if (eigensolver.info() != Eigen::Success)
            return;

        eigenvectors = eigensolver.eigenvectors();
        eigenvalues = eigensolver.eigenvalues();
    }

    CloudPtrT projectOnLocalPlane(CloudPtrT cloud, std::vector<int> indices,
                                  Eigen::Vector4f plane)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0] = plane(0);
        coefficients->values[1] = plane(1);
        coefficients->values[2] = plane(2);
        coefficients->values[3] = plane(3);

        // extract the given indices as point cloud
        CloudPtrT extract(new CloudT);
        pcl::ExtractIndices<PointInT> extractor;

        boost::shared_ptr<std::vector<int>> indices_ptr = boost::make_shared
            <std::vector<int>>(indices);
        extractor.setInputCloud(cloud);
        extractor.setIndices(indices_ptr);
        extractor.filter(*extract);

        CloudPtrT projected(new CloudT);

        // Create the filtering object
        pcl::ProjectInliers<PointInT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*projected);

        return projected;
    }

    float computeIntensityStandardDeviation(const CloudPtrT in_cloud,
                                            const std::vector<int> indices)
    {
        float avg = computeIntensityMean(in_cloud, indices);
        float std = 0.0;

        float tmp;
        BOOST_FOREACH(int i, indices)
        {
            tmp = in_cloud->at(i).intensity - avg;
            std += tmp * tmp;
        }

        std /= indices.size();
        std = sqrt(std);
        return std;
    }

    float computeIntensityMean(const CloudPtrT in_cloud,
                               const std::vector<int> &indices)
    {
        // compute the average
        float avg = 0.0;
        for (int i = 0; i < indices.size(); ++i)
            avg += in_cloud->at(i).intensity;

        avg /= indices.size();
        return avg;
    }

    void fitLine(CloudPtrT cloud, std::vector<int> indices, float &lam1,
                 float &lam2, Eigen::Vector3f &vector)
    {
        Eigen::Matrix3f covmat, eigenvector;
        Eigen::Vector3f eigenvalues;
        spc::computeCovMat<PointInT>(*cloud, indices, covmat);
        solveEigenvectorEigenvalues(covmat, eigenvector, eigenvalues);

        lam1 = eigenvalues(1);
        lam2 = eigenvalues(2); // associated with the direction of the line

        vector(0) = eigenvector(0, 2);
        vector(1) = eigenvector(1, 2);
        vector(2) = eigenvector(2, 2);
    }

    float getDistanceFromCenterOfMass(const PointInT &point,
                                      const CloudPtrT cloud_in,
                                      const std::vector<int> indices)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid<PointInT>(*cloud_in, indices, centroid);

        PointInT c;
        c.x = centroid(0);
        c.y = centroid(1);
        c.z = centroid(2);

        return pcl::euclideanDistance(point, c);
    }

private:
    CloudPtrT in_cloud_;

    std::map<float, CloudPtrT> downsampled_versions_;

    std::map<CloudPtrT, FlannSearchPtrT> searchers_;

    std::vector<float> scales_;

    bool use_only_full_size_;
};

} // end namespace

#endif // KEYPOINTS_EXTRACTOR_H
