#include <spc/calibration/RBFEvaluator.h>


#include "ceres/ceres.h"
#include "glog/logging.h"



using namespace spc;
const int kNumObservations = 67;

double data[] = {
    0.000000e+00, 1.133898e+00,
    7.500000e-02, 1.334902e+00,
    1.500000e-01, 1.213546e+00,
    2.250000e-01, 1.252016e+00,
    3.000000e-01, 1.392265e+00,
    3.750000e-01, 1.314458e+00,
    4.500000e-01, 1.472541e+00,
    5.250000e-01, 1.536218e+00,
    6.000000e-01, 1.355679e+00,
    6.750000e-01, 1.463566e+00,
    7.500000e-01, 1.490201e+00,
    8.250000e-01, 1.658699e+00,
    9.000000e-01, 1.067574e+00,
    9.750000e-01, 1.464629e+00,
    1.050000e+00, 1.402653e+00,
    1.125000e+00, 1.713141e+00,
    1.200000e+00, 1.527021e+00,
    1.275000e+00, 1.702632e+00,
    1.350000e+00, 1.423899e+00,
    1.425000e+00, 1.543078e+00,
    1.500000e+00, 1.664015e+00,
    1.575000e+00, 1.732484e+00,
    1.650000e+00, 1.543296e+00,
    1.725000e+00, 1.959523e+00,
    1.800000e+00, 1.685132e+00,
    1.875000e+00, 1.951791e+00,
    1.950000e+00, 2.095346e+00,
    2.025000e+00, 2.361460e+00,
    2.100000e+00, 2.169119e+00,
    2.175000e+00, 2.061745e+00,
    2.250000e+00, 2.178641e+00,
    2.325000e+00, 2.104346e+00,
    2.400000e+00, 2.584470e+00,
    2.475000e+00, 1.914158e+00,
    2.550000e+00, 2.368375e+00,
    2.625000e+00, 2.686125e+00,
    2.700000e+00, 2.712395e+00,
    2.775000e+00, 2.499511e+00,
    2.850000e+00, 2.558897e+00,
    2.925000e+00, 2.309154e+00,
    3.000000e+00, 2.869503e+00,
    3.075000e+00, 3.116645e+00,
    3.150000e+00, 3.094907e+00,
    3.225000e+00, 2.471759e+00,
    3.300000e+00, 3.017131e+00,
    3.375000e+00, 3.232381e+00,
    3.450000e+00, 2.944596e+00,
    3.525000e+00, 3.385343e+00,
    3.600000e+00, 3.199826e+00,
    3.675000e+00, 3.423039e+00,
    3.750000e+00, 3.621552e+00,
    3.825000e+00, 3.559255e+00,
    3.900000e+00, 3.530713e+00,
    3.975000e+00, 3.561766e+00,
    4.050000e+00, 3.544574e+00,
    4.125000e+00, 3.867945e+00,
    4.200000e+00, 4.049776e+00,
    4.275000e+00, 3.885601e+00,
    4.350000e+00, 4.110505e+00,
    4.425000e+00, 4.345320e+00,
    4.500000e+00, 4.161241e+00,
    4.575000e+00, 4.363407e+00,
    4.650000e+00, 4.161576e+00,
    4.725000e+00, 4.619728e+00,
    4.800000e+00, 4.737410e+00,
    4.875000e+00, 4.727863e+00,
    4.950000e+00, 4.669206e+00,
};

//template class spc::RBFEvaluator<ceres::Jet<double, 6>>;




struct RBFResidual {
    RBFResidual(double x, double y, const Eigen::MatrixXd &knots_): x_(x), y_(y), knots_(knots_)
    {

    }

    template <typename T> bool operator()(const T* const coeffs, T* residual) const
    {

        for (int i =0; i <6;++i)
        {
            std::cout << coeffs[i] << std::endl;
        }


        Eigen::Matrix<T, 6, 1> c = Eigen::Map<const Eigen::Matrix<T, 6, 1>>(coeffs);


//        std::cout <<"evaluating with coefficients: " << c << std::endl;

         RBFEvaluator<T> model;


        model.getParameters()->setCoefficients(c);

        typename RBFKernelGaussian<T>::Ptr k (new RBFKernelGaussian<T>(T(1.0)));

        model.getParameters()->setKernel(k);
        model.getParameters()->setNodes(knots_.cast<T>());
//        model.getParameters()->initUnityCoefficients();


        Eigen::Matrix<T, 1, 1> eigx;
        eigx << T(x_);
        T predicted = model(eigx);


        std::cout << "predicted "<< predicted << std::endl;



        residual[0] = T(y_) - predicted;

        std::cout << "residual " << *residual << std::endl;
        return true;
    }

private:
    const double x_;
    const double y_;

    Eigen::MatrixXd knots_;
};



int main (int argc, char ** argv)
{
    typedef Eigen::Map<Eigen::Matrix<double,kNumObservations,2,Eigen::RowMajor>> MatrixMapT;
    MatrixMapT edata = MatrixMapT (data, kNumObservations, 2);

    std::cout << edata << std::endl;

    Eigen::Matrix<double, 6, 1> knots;
    knots << 0,1,2,3,4,5;


    google::InitGoogleLogging(argv[0]);


    ceres::Problem problem;


    double coeffs[] = {100,10,1,10,1,1};


    for (int i = 0 ; i < kNumObservations; ++i)
    {



        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<RBFResidual, 1, 6>(
                                     new RBFResidual(edata(i,0), edata(i,1), knots)),
                                 NULL,
                                 coeffs);

        std::cout <<"set up cost with values: " << edata.row(i) << std::endl;


    }


    ceres::Jet<double, 6> j(0);


    std::cout << "test jet "<< j << std::endl;

    std::cout << "test jet sqrt "<< sqrt(j) << std::endl;






    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.function_tolerance = 1e-18;
    options.gradient_tolerance = 1e-18;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
//    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
//    std::cout << "Final   m: " <<  parameters << std::endl;

    std::cout << "FINAL: "<<  std::endl;

    Eigen::VectorXd eigpars = Eigen::Map<Eigen::Matrix<double, 6, 1>> (coeffs);

    std::cout << eigpars << std::endl;



//    for (int i =0; i < 6; ++i)
//        std::cout << coeffs[i] << std::endl;

   RBFEvaluator<> model;


   model.getParameters()->setCoefficients(eigpars);

   typename RBFKernelGaussian<>::Ptr k (new RBFKernelGaussian<>(1.0));

   model.getParameters()->setKernel(k);
   model.getParameters()->setNodes(knots);

   auto prediction = model.batchEval(edata.col(0));

   std::cout << "prediction:\n" << prediction<< std::endl;


    return 1;
}
