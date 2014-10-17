#ifndef LINEARCALIBRATOR_H
#define LINEARCALIBRATOR_H

#include <spc/calibration/SampledData.h>

namespace spc
{

class LinearCalibrator
{
public:
    LinearCalibrator(const std::string & filename)
    {
        data_.readFile(filename);

        Eigen::VectorXf tmp_dists = Eigen::VectorXf::LinSpaced(dist_n_nodes_, data_.d_.minCoeff(), data_.d_.maxCoeff());

        Eigen::VectorXf tmp_angles = Eigen::VectorXf::LinSpaced(angle_n_nodes_, data_.a_.minCoeff(), data_.a_.maxCoeff());

        distance_sigma_ = tmp_dists(1) - tmp_dists(0);
        angle_sigma_ = tmp_angles(1) - tmp_angles(0);

        getMinMax();

        setUpNodes();


        points_.resize(data_.d_.size(), 2);

        points_.col(0) = d_scaled_;
        points_.col(1) = data_.a_;

        b_ = data_.i_;


        buildAMatrix();

        solve();
    }



    void setUpNodes()
    {


        Eigen::VectorXf dist_nodes = Eigen::VectorXf::LinSpaced(dist_n_nodes_, min_distance_scaled_, max_distance_scaled_);
        std::cout << "Nodes for distace: " << dist_nodes.transpose() << std::endl;

        Eigen::VectorXf angle_nodes = Eigen::VectorXf::LinSpaced(angle_n_nodes_, min_angle_, max_angle_);
        std::cout << "Nodes for angle: " << angle_nodes.transpose() << std::endl;


        std::cout << "A total of " << dist_n_nodes_ * angle_n_nodes_ << " nodes will be used" << std::endl ;

//        // set up kernel sizes
//        distance_sigma_ = (dist_nodes(1) - dist_nodes(0)) * 4;
//        angle_sigma_ = angle_nodes(1) - angle_nodes(0);




        Eigen::MatrixXf rep_dist = dist_nodes.replicate(angle_n_nodes_, 1);
        Eigen::MatrixXf rep_angle = angle_nodes.replicate(1, dist_n_nodes_).transpose();


        nodes_.resize(rep_dist.rows(), 2);
        nodes_.col(0) = rep_dist;
        nodes_.col(1) = rep_angle.array();

        std::cout << "Matrix of nodes" << std::endl;
        std::cout << nodes_ << std::endl;


    }

    Eigen::VectorXf getSquaredDistancePointToNodes(const Eigen::VectorXf &point) const
    {
        // compute squared distances of point from nodes
        Eigen::Matrix<float, -1, -1> diff  = nodes_.rowwise() - point.transpose();
        Eigen::Matrix<float, -1, 1> sq_dist =  diff.rowwise().squaredNorm();

        return sq_dist;
    }

    Eigen::VectorXf getWeightsForPoint(const Eigen::VectorXf &point) const
    {
        Eigen::VectorXf sq_dist = getSquaredDistancePointToNodes(point);

        // transform these sq_dist in weights
        Eigen::VectorXf weights(sq_dist.rows());

        for (int i = 0; i < sq_dist.rows(); ++i)
            weights(i)  = exp(- sq_dist(i) / (squared_angle_sigma_)  );

        return weights;
    }

    Eigen::MatrixXf getWeightsForPoints(const Eigen::MatrixXf &points) const
    {

        Eigen::MatrixXf out;
        out.resize(points.rows(), dist_n_nodes_ * angle_n_nodes_);

        for (int i = 0 ; i < points.rows(); ++i)
        {
            out.row(i) = getWeightsForPoint(points.row(i));
        }

        return out;
    }

    void buildAMatrix()
    {

        A_ = getWeightsForPoints(points_);
    }

    void solve()
    {
        coeffs_  = A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_);

        std::cout << "coeffs:" << std::endl;
        std::cout << coeffs_ << std::endl;
    }


    void getMinMax()
    {

        std::cout << "Computing min-max" << angle_sigma_ <<std::endl;
        sigma_ratio_ = angle_sigma_ / distance_sigma_;
        squared_angle_sigma_  = angle_sigma_ * angle_sigma_;

        d_scaled_ = data_.d_.array() * sigma_ratio_;

        min_distance_ = data_.d_.minCoeff();
        max_distance_ = data_.d_.maxCoeff();

        min_distance_scaled_ = d_scaled_.minCoeff();
        max_distance_scaled_ = d_scaled_.maxCoeff();

        min_angle_ = data_.a_.minCoeff();
        max_angle_ = data_.a_.maxCoeff();

        std::cout << "Distance in (" << min_distance_ <<", " << max_distance_ << ")" << std::endl ;
        std::cout << "Angles in (" << min_angle_ <<", " << max_angle_ << ")" << std::endl ;

    }

    Eigen::VectorXf getPredictionForInput() const
    {

        return A_ * coeffs_;
    }

    SampledData & getData()
    {
        return data_;
    }

protected:

    SampledData data_;

    Eigen::MatrixXf nodes_;
    Eigen::MatrixXf points_;
    Eigen::VectorXf coeffs_;
    Eigen::VectorXf b_;
    Eigen::MatrixXf A_;

    Eigen::VectorXf d_scaled_;

    size_t dist_n_nodes_ = 40;
    size_t angle_n_nodes_ = 40;


    // we use an anisotropic kernel
    // in this way we can specify different kernel sizes for distace and angle
    float distance_sigma_ = 10;
    float angle_sigma_ = 10;
    float squared_angle_sigma_;
    float sigma_ratio_;


    float min_distance_;
    float max_distance_;
    float min_distance_scaled_;
    float max_distance_scaled_;
    float min_angle_;
    float max_angle_;




};

}

#endif // LINEARCALIBRATOR_H
