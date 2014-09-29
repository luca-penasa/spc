#include <spc/calibration/RBFEvaluator.h>


#include "ceres/ceres.h"
#include "glog/logging.h"

#include <spc/io/element_io.h>
#include <spc/elements/EigenTable.h>

#include <spc/io/AsciiEigenTableWriter.h>

using namespace spc;
using Eigen::Matrix;

//template class spc::RBFEvaluator<ceres::Jet<double, 6>>;


template <typename T>
T compute_rbf(const Matrix<T, -1, 1> &coefficients,
              const Matrix<T, -1, -1> &nodes,
              const T &sigma,
              const Matrix<T, -1, 1> &point)
{



    // compute squared distances of point from nodes
    Matrix<T, -1, -1> diff  = nodes.rowwise() - point.transpose();
    Matrix<T, -1, 1> sq_dist =  diff.rowwise().squaredNorm();

   // transform these sq_dist in weights
    Matrix<T, -1, 1> weights(sq_dist.rows());

    for (int i = 0; i < sq_dist.rows(); ++i)
        weights(i)  = exp(- sq_dist(i) / (sigma * sigma)  );

    // compute the RBF value
    return weights.cwiseProduct(coefficients ).sum();
}

template <size_t NDISTPARS, size_t NANGLEPARS>
struct ResidualFunctor
{
    ResidualFunctor(double distance,
                    double angle,
                    double intensity,
                    double sigma_dist,
                    double sigma_angle,
                    Eigen::MatrixXd * knots_dist,
                    Eigen::MatrixXd * knots_angle):
        distance_(distance),
        angle_(angle),
        intensity_(intensity),
        sigma_dist_(sigma_dist),
        sigma_angle_(sigma_angle)

    {
        knots_dist_ = knots_dist;
        knots_angle_ = knots_angle;
//        std::cout  << "sigmas: " << distance << " " << angle << std::endl;
    }

    template <typename T> bool operator()(const T* const coeffs_dist, const T* const coeffs_angle, T* residual) const
    {

        Eigen::Matrix<T, NDISTPARS, 1> c_dist = Eigen::Map<const Eigen::Matrix<T, NDISTPARS, 1>>(coeffs_dist);

        Eigen::Matrix<T, NANGLEPARS, 1> c_angle = Eigen::Map<const Eigen::Matrix<T, NANGLEPARS, 1>>(coeffs_angle);



        Eigen::Matrix<T, 1, 1> edist;
        edist << T(distance_);

        Eigen::Matrix<T, 1, 1> eang;
        eang<< T(angle_);


        T d_effect = compute_rbf<T>(c_dist, (*knots_dist_).template cast<T>(), T(sigma_dist_), edist );
        T a_effect = compute_rbf<T>(c_angle, (*knots_angle_).template cast<T>(), T(sigma_angle_), eang );


        T prediction = d_effect * a_effect;

        double alpha = 0;

        residual[0] = T(intensity_) - prediction;

        for (int i = 0 ; i< NDISTPARS; ++i)
        {
            residual[i+1] = coeffs_dist[i] * alpha;
        }

        for (int i = 0;i< NANGLEPARS; ++i)

        {
            residual[i+1+NDISTPARS] = coeffs_angle[i] * alpha;
        }

        return true;
    }

private:
    const double distance_;
    const double angle_;
    const double intensity_;
    const double sigma_dist_;
    const double sigma_angle_;


    Eigen::MatrixXd * knots_dist_;
    Eigen::MatrixXd * knots_angle_;

};

//! an unique that should work with eigen types
template <typename ObjT>
ObjT unique(const ObjT& b)
{
    ObjT tmp = b;


    std::sort(tmp.data(), tmp.data() + tmp.size());
    auto last = std::unique(tmp.data(), tmp.data() + tmp.size());

    size_t n_elements = last - tmp.data();

    tmp.conservativeResize(n_elements);

    return tmp;
}


int main (int argc, char ** argv)
{
    std::string datadb = "/home/luca/Desktop/calibration_db_small.spc";

    ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

    EigenTable::Ptr table = spcDynamicPointerCast<EigenTable> (o);

    table = table->getWithStrippedNANs({"distance", "intensity", "angle", "intensity_std"});

    std::cout << "Following columns found:\n" << std::endl;
    for (int i = 0; i < table->getNumberOfColumns(); ++i)
    {
        std::cout << table->getColumnName(i) << std::endl;
    }

    Eigen::VectorXf i = table->mat().col(table->getColumnId("intensity"));
    Eigen::VectorXf d = table->mat().col(table->getColumnId("distance"));
    Eigen::VectorXf a = table->mat().col(table->getColumnId("angle"));

    Eigen::VectorXf i_std = table->mat().col(table->getColumnId("intensity_std"));

    Eigen::VectorXi cloud_ids = table->mat().col(table->getColumnId("cloud_id")).cast<int>();
    Eigen::VectorXi c = unique(cloud_ids);

    std::cout << "unique cloud ids found: \n" << c << std::endl;


//    std::cout << cloud_ids << std::endl;
    /////////////////////////////////////////////////////

    google::InitGoogleLogging(argv[0]);

    ceres::Problem problem;


    //////////////////////////////////////////////////

    const int n_dist_pars = 6;
    const int n_ang_pars = 4;

    Eigen::MatrixXd * knots_distance =  new Eigen::MatrixXd(Eigen::VectorXd::LinSpaced (n_dist_pars, d.minCoeff(), d.maxCoeff()));

    Eigen::MatrixXd * knots_angles =  new Eigen::MatrixXd(Eigen::VectorXd::LinSpaced (n_ang_pars, 0, 90));

//    *knots_angles << 0.,  30, 60, 90;

    std::cout << "node for distance " << std::endl;
    std::cout << *knots_distance << std::endl;

    std::cout << "node for angle " << std::endl;
    std::cout << *knots_angles << std::endl;


    double coeffs_dist[n_dist_pars];
    for (int i = 0; i < n_dist_pars; ++i)
        coeffs_dist[i] = 1;

    double dist_sigma = knots_distance->operator() (1) - knots_distance->operator ()(0);

    double coeffs_angle[n_ang_pars];
    for (int i = 0; i < n_ang_pars; ++i)
        coeffs_angle[i] = 1;

    double angle_sigma = knots_angles->operator() (1) - knots_angles->operator ()(0);



    for (int j = 0 ; j < i.rows(); ++j)
    {
        ResidualFunctor<n_dist_pars,n_ang_pars> * f = new ResidualFunctor<n_dist_pars,n_ang_pars>( d(j), a(j), i(j) , dist_sigma, angle_sigma, knots_distance, knots_angles);

        problem.AddResidualBlock(new ceres::AutoDiffCostFunction< ResidualFunctor<n_dist_pars,n_ang_pars>, 1 + n_ang_pars + n_dist_pars, n_dist_pars, n_ang_pars> ( f ),
                                 new ceres::ScaledLoss(NULL, 1/(i_std(j)*i_std(j)), ceres::TAKE_OWNERSHIP),
//                                 NULL,
//                                 HuberLoss(0.00001),
                                 coeffs_dist,
                                 coeffs_angle);

    }


    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.function_tolerance = 1e-6;
    options.gradient_tolerance = 1e-6;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";



    std::cout << "pars dist:" << std::endl;
    for (int i =0; i <n_dist_pars; ++i)
    {
        std::cout << coeffs_dist[i] << std::endl;
    }



    std::cout << "pars angle:" << std::endl;
    for (int i =0; i <n_ang_pars; ++i)
    {
        std::cout << coeffs_angle[i] << std::endl;
    }



    /////////////////////////////////////////////// recomputing

    Eigen::VectorXd eig_dist_coeffs = Eigen::Map<Matrix<double, n_dist_pars, 1>> (coeffs_dist);

    Eigen::VectorXd eig_angle_coeffs = Eigen::Map<Matrix<double, n_ang_pars, 1>> (coeffs_angle);


    Eigen::VectorXd predicted(i.rows());
    Eigen::VectorXd dist_effect(i.rows());
    Eigen::VectorXd angle_effect(i.rows());


    for (int j = 0; j < i.rows(); ++j)
    {
        Eigen::Matrix<double, -1, 1> distance(1);
        distance << d(j);

        Eigen::Matrix<double, -1, 1> angle(1);
        angle << a(j);

        double valdist = compute_rbf<double>(eig_dist_coeffs, *knots_distance, dist_sigma, distance);
        double valang = compute_rbf<double>(eig_angle_coeffs, *knots_angles, angle_sigma, angle);

        predicted(j) = valdist * valang;
        dist_effect(j) = valdist;
        angle_effect(j) = valang;
    }

    EigenTable::Ptr out (new EigenTable);
    out->addNewComponent("distance", 1);
    out->addNewComponent("angle", 1);
    out->addNewComponent("intensity", 1);
    out->addNewComponent("pred_intensity", 1);
    out->addNewComponent("angle_effect", 1);
    out->addNewComponent("dist_effect", 1);

    out->resize(i.rows());

    out->column("distance") = d;
    out->column("angle") = a;
    out->column("intensity") = i;
    out->column("pred_intensity") = predicted.cast<float>();
    out->column("dist_effect") = dist_effect.cast<float>();
    out->column("angle_effect") = angle_effect.cast<float>();

    spc::io::AsciiEigenTableWriter w;
    w.setInput(out);
    w.setOutputFilename("/home/luca/test.txt");
    w.setWriteHeaders(true);
    w.write();

    return 1;
}
