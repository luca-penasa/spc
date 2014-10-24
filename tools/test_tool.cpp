
#include <iostream>
#include <spc/methods/spc_eigen.h>
#include <Eigen/Sparse>
#include <unsupported/Eigen/AutoDiff>

#include <spc/methods/polynomials.hpp>

#include <spc/methods/RBFModelEstimator.h>

#include <spc/elements/Kernels.hpp>
#include <spc/io/element_io.h>


int main()
{

//    spc::BasicKernel<float>::Ptr a (new spc::GaussianKernel<float>(1));

//    std::cout << a->operator ()(2) << std::endl;

    spc::RBFModelEstimator<float> interp;



    Eigen::Matrix<float, -1, -1> points = Eigen::MatrixXf::Random(10, 3);
    Eigen::VectorXf values = Eigen::VectorXf::Random(10);

    Eigen::Matrix<float, -1, -1> nodes = Eigen::MatrixXf::Random(8, 3);

    Eigen::VectorXf weights = Eigen::MatrixXf::Random(10, 1);



    std::cout << "points\n" << points <<std::endl;

    std::cout << "values\n" << values <<std::endl;


    interp.setPoints(points);
    interp.getModel()->setNodes(nodes);
    interp.getModel()->setPolyOrder(1);
    interp.getModel()->setSigma(1);

//    interp.setWeights(weights);

    Eigen::VectorXf scales(3);
    scales << 1, 1, 1;

    std::cout << "\n scales \n" << scales;
    interp.getModel()->setScales(scales);
    interp.setInputValues(values);
    interp.setLambda(0.001);
    interp.solveProblem();

    spc::RBFModel<float>::Ptr model = interp.getModel();

    std::cout << "\n parameters \n" << model->getCoefficients().transpose() << std::endl;


    spc::io::serializeToFile(model, "/home/luca/model", spc::io::XML);


    Eigen::VectorXf out;
    std::cout << "evaluating" << std::endl;
    (*model)(points, out);
    std::cout << "done" << std::endl;

    std::cout <<"\n predicted \n" << out << std::endl;

    Eigen::MatrixXf splitted;
    model->splittedEvaluator(points, splitted);

    std::cout << "\n splitted: \n" << splitted;

    std::cout <<"\n sum of splitted \n " << splitted.rowwise().sum() << std::endl;


}
