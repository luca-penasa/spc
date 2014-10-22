
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/AutoDiff>

#include <spc/methods/polynomials.hpp>

#include <spc/methods/InterpolatorRBF.hpp>

#include <spc/methods/Kernels.hpp>


int main()
{

//    spc::BasicKernel<float>::Ptr a (new spc::GaussianKernel<float>(1));

//    std::cout << a->operator ()(2) << std::endl;

    spc::InterpolatorRBF<float> interp;


    Eigen::Matrix<float, -1, -1> points;
    points.resize(4, 3);

    points << 1, 2, 1,
              4, 5, 2,
              2, 2, 5,
              1, 1, 6;


    Eigen::Matrix<float, -1, -1> nodes;
    nodes.resize(3, 3);

    nodes << 1, 2, 2,
            2,3,4,
            4,5,6;


    Eigen::VectorXf values(4);
    values << 1, 4, 11, 22;

    std::cout << "points\n" << points <<std::endl;

    interp.setPoints(points);
//    interp.setNodes(nodes);
    interp.setPolyOrder(0);
    interp.setSigma(1);
    interp.setInputValues(values);
//    interp.setLambda(10);
    interp.solveProblem();


    float val = interp.evaluate(points.row(2));
    std::cout <<"\n values \n" << val << std::endl;
}
