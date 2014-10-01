#ifndef CALIBRATIONMANAGER_H
#define CALIBRATIONMANAGER_H

#include <spc/elements/ElementBase.h>
#include <spc/calibration/Observations.h>
#include <spc/calibration/CalibrationFactors.h>
#include <spc/calibration/ResidualBlocks.h>
#include <spc/elements/EigenTable.h>
#include <ceres/ceres.h>

#include <spc/io/AsciiEigenTableWriter.h>
#include <spc/io/element_io.h>


#include <spc/calibration/ModelingFunctions.h>
namespace spc
{

class CalibratorManager
{
public:
    CalibratorManager(char ** argv);

    void setSolverOptions();

    void setUpFixedPars()
    {
        fixed_pars_.n_dist_pars = 6;
        fixed_pars_.n_ang_pars = 4;

        fixed_pars_.knots_dist =  new Eigen::MatrixXd(Eigen::VectorXd::LinSpaced (fixed_pars_.n_dist_pars, d_.minCoeff(), d_.maxCoeff()));

        fixed_pars_.knots_angle =  new Eigen::MatrixXd(Eigen::VectorXd::LinSpaced (fixed_pars_.n_ang_pars, 0, 90));


        std::cout << "node for distance " << std::endl;
        std::cout << *fixed_pars_.knots_dist << std::endl;

        std::cout << "node for angle " << std::endl;
        std::cout << *fixed_pars_.knots_angle << std::endl;


        fixed_pars_.sigma_dist = fixed_pars_.knots_dist->operator() (1) - fixed_pars_.knots_dist->operator ()(0);
        fixed_pars_.sigma_angle= fixed_pars_.knots_angle->operator() (1) - fixed_pars_.knots_angle->operator ()(0);



    }

    void setUpInitParametersBlocks()
    {

        std::cout << "setting up blocks" << std::endl;

        dist_pars_ei_ = Eigen::VectorXd::Ones(fixed_pars_.n_dist_pars);
        angle_pars_ei_ = Eigen::VectorXd::Ones(fixed_pars_.n_ang_pars);

        std::cout << "init pars - dist: " << std::endl;
        std::cout << dist_pars_ei_ << std::endl;

        std::cout << "init pars - ang: " << std::endl;
        std::cout << angle_pars_ei_ << std::endl;

        parameters_.push_back(dist_pars_ei_.data());
        parameters_.push_back(angle_pars_ei_.data());


    }

    void setUpProblem()
    {


        for (int j = 0 ; j < obs_.size(); ++j)
        {

            Observation* ob = &obs_.at(j);
            ResidualFunctor * f = new ResidualFunctor( ob, &fixed_pars_);

            ceres::DynamicAutoDiffCostFunction<ResidualFunctor, 4> * cost  = new ceres::DynamicAutoDiffCostFunction< ResidualFunctor, 4> ( f );

            cost->AddParameterBlock(fixed_pars_.n_dist_pars);
            cost->AddParameterBlock(fixed_pars_.n_ang_pars);
            cost->SetNumResiduals(1);

//            std::cout << "BLOCKS: " << parameters_.size() << std::endl;



            problem_.AddResidualBlock(cost,
                                     //                                 new ceres::ScaledLoss(NULL, 1/(i_std(j)*i_std(j)), ceres::TAKE_OWNERSHIP),
                                     NULL,
                                     //                                 HuberLoss(0.00001),
                                     parameters_
                                     );

        }

        //////// SMOOTHNESS CONSTRAIN ///////////
        SmoothnessConstrain * smoothness = new SmoothnessConstrain(0.01 * obs_.size(), &fixed_pars_);
        ceres::DynamicAutoDiffCostFunction<SmoothnessConstrain, 4> * cost2  = new ceres::DynamicAutoDiffCostFunction< SmoothnessConstrain, 4> ( smoothness );

        cost2->AddParameterBlock(fixed_pars_.n_dist_pars);
        cost2->AddParameterBlock(fixed_pars_.n_ang_pars);
        cost2->SetNumResiduals(smoothness->getNumberOfResiduals());

        problem_.AddResidualBlock(cost2, NULL, parameters_);

    //    problem.GetResidualBlocks();


    }

    void readFile(const std::string & fname);

    void solve();

    void printFullReport()
    {
        std::cout << summary_.FullReport() << "\n";

        std::cout << "final pars - dist: " << std::endl;
        std::cout << dist_pars_ei_ << std::endl;

        std::cout << "final pars - ang: " << std::endl;
        std::cout << angle_pars_ei_ << std::endl;


    }

    void savePrediction(const std::string outfname) const;

    EigenTable::Ptr table_;

    Eigen::VectorXf d_;
    Eigen::VectorXf a_;
    Eigen::VectorXf i_;
    Eigen::VectorXi cloud_ids_;
    Eigen::VectorXi unique_ids_;

    std::vector<Observation> obs_;

    ceres::Solver::Options options_;


    ceres::Solver::Summary summary_;


    ceres::Problem problem_;



    FixedModelPars fixed_pars_ ;


    std::vector<double *> parameters_;

    Eigen::VectorXd dist_pars_ei_;
    Eigen::VectorXd angle_pars_ei_ ;

};

}//end nspace
#endif // CALIBRATIONMANAGER_H
