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
//    std::cout << diff(9,0) << std::endl;


    Matrix<T, -1, 1> sq_dist =  diff.rowwise().squaredNorm();

//    std::cout << sq_dist(9) << std::endl;


    // transform these sq_dist in weights
    Matrix<T, -1, 1> weights(sq_dist.rows());

    for (int i = 0; i < sq_dist.rows(); ++i)
        weights(i)  = exp(- sq_dist(i) / (sigma * sigma)  );

//    std::cout <<"w: "<<  weights(0) <<std::endl;

    // compute the RBF value
    return weights.cwiseProduct(coefficients ).sum();
}

struct ResidualFunctor {
    ResidualFunctor(double distance, double intensity, Eigen::MatrixXd * knots): distance_(distance), intensity_(intensity)
    {

        knots_ = knots;
    }

    template <typename T> bool operator()(const T* const coeffs, T* residual) const
    {

        Eigen::Matrix<T, 10, 1> c = Eigen::Map<const Eigen::Matrix<T, 10, 1>>(coeffs);

        Eigen::Matrix<T, 1, 1> eigx;
        eigx << T(distance_);


        T predicted = compute_rbf<T>(c, (*knots_).template cast<T>(),T(10), eigx );

//        std::cout << "predicted " << predicted << std::endl;


        residual[0] = T(intensity_) - predicted;
        return true;
    }

private:
    const double distance_;
    const double intensity_;

    Eigen::MatrixXd * knots_;


};




int main (int argc, char ** argv)
{
    std::string datadb = "/home/luca/Desktop/calibration_db.spc";

    ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

    EigenTable::Ptr table = spcDynamicPointerCast<EigenTable> (o);

    table = table->getWithStrippedNANs({"distance", "intensity", "angle"});

    std::cout << "Following columns found:\n" << std::endl;
    for (int i = 0; i < table->getNumberOfColumns(); ++i)
    {
        std::cout << table->getColumnName(i) << std::endl;
    }

    Eigen::VectorXf i = table->mat().col(table->getColumnId("intensity"));



    Eigen::VectorXf d = table->mat().col(table->getColumnId("distance"));
    Eigen::VectorXf a = table->mat().col(table->getColumnId("angle"));

    //   std::cout << a << std::endl;

    google::InitGoogleLogging(argv[0]);

    ceres::Problem problem;

    Eigen::MatrixXd * knots =  new Eigen::MatrixXd (10, 1);
    *knots << 0, 15, 30, 45, 60, 75, 90, 105, 120, 135;


    double coeffs[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    for (int j = 0 ; j < i.rows(); ++j)
    {

//        if (std::isfinite(i(j)) & std::isfinite(d(j)))
//        {
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ResidualFunctor, 1, 10>(
                                         new ResidualFunctor(d(j), i(j) , knots)),
                                     NULL,
                                     coeffs);

//            std::cout << i(j) << std::endl;
//        }



    }


    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.function_tolerance = 1e-18;
    options.gradient_tolerance = 1e-18;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";


    Eigen::VectorXd eig_coeffs = Eigen::Map<Matrix<double, 10, 1>> (coeffs);

    std::cout << "pars:" << std::endl;
    for (int i =0; i <10; ++i)
    {
        std::cout << coeffs[i] << std::endl;
    }

    Eigen::VectorXd predicted(i.rows());

    for (int j = 0; j < i.rows(); ++j)
    {
        Eigen::Matrix<double, -1, 1> point(1);
        point << d(j);

//       std::cout << point << std::endl;
        double val = compute_rbf<double>(eig_coeffs, *knots, 10.0, point);
        predicted(j) = val;
//       std::cout << val << std::endl;
    }

    EigenTable::Ptr out (new EigenTable);
    out->addNewComponent("distance", 1);
    out->addNewComponent("intensity", 1);
    out->addNewComponent("pred_intensity", 1);

    out->resize(i.rows());

    out->column("distance") = d;
    out->column("intensity") = i;
    out->column("pred_intensity") = predicted.cast<float>();

    spc::io::AsciiEigenTableWriter w;
    w.setInput(out);
    w.setOutputFilename("/home/luca/test.txt");
    w.setWriteHeaders(true);
    w.write();

    return 1;
}
