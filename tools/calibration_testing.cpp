#include <spc/calibration/SimpleRBF.h>
using namespace spc;


int main (int argc, char ** argv)
{
    spc::SimpleRBF a;

    Eigen::MatrixXf knots(3,2);
    knots << 1,2 ,
            2,3 ,
            4,5;


    Eigen::VectorXf pars(3);

    pars << 1,1,1 ;
    std::cout << "knotws:\n" << knots << std::endl;
    std::cout << "pars:\n" << pars << std::endl;





    a.setKnots(knots);
    a.setParameters(pars);


    Eigen::VectorXf d = a.getDistancesFromKnots(knots.row(0));

    Eigen::VectorXf w = a.getWeightsAtPoint(knots.row(0));

    std::cout << "distance from knots:\n" << d<< std::endl ;
    std::cout << "and weights:\n" << w<< std::endl ;


    Eigen::VectorXf newv= knots.row(0);

    std::cout << "simensionlity " << a.getDimensions() << std::endl;
    std::cout << "simensionlity " << newv.rows() << std::endl;

    float value = a(knots.row(0));

    std::cout << "value:" <<value << std::endl;



    return 1;
}
