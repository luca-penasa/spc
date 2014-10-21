
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/AutoDiff>

#include <spc/methods/polynomials.hpp>

#include <spc/methods/InterpolatorRBF.hpp>


int main()
{
    spc::InterpolatorRBF<float, 3> interp;


    Eigen::Matrix<float, -1, -1> points;
    points.resize(4, 3);

    points << 1, 2, 1,
              4, 5, 2,
              2, 2, 5,
              1, 1, 6;


    Eigen::Matrix<float, -1, 3> p2 = points;



    Eigen::Matrix<float, -1, -1> nodes;
    nodes.resize(3, 3);

    nodes << 1, 2, 2,
            2,3,4,
            4,5,6;



    Eigen::Matrix<float, -1, 3> n2 = nodes;


    Eigen::VectorXf values(4);
    values << 1, 4, 11, 22;

    std::cout << "points\n" << points <<std::endl;

    interp.setPoints(p2);
//    interp.setNodes(n2);
    interp.setPolyOrder(0);
    interp.setSigma(10);
    interp.setInputValues(values);
    interp.setLambda(0);
    interp.initProblem();
    interp.solveProblem();


    float val = interp.evaluate(p2.row(2));
    std::cout <<"\n values \n" << val << std::endl;
}
