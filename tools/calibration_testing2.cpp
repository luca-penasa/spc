#include <spc/calibration/RBFEvaluator.h>


#include "ceres/ceres.h"
#include "glog/logging.h"

#include <spc/io/element_io.h>
#include <spc/elements/EigenTable.h>

#include <spc/io/AsciiEigenTableWriter.h>

using namespace spc;
using Eigen::Matrix;

//template class spc::RBFEvaluator<ceres::Jet<double, 6>>;

class FixedModelPars
{
public:
    double sigma_dist;
    double sigma_angle;
    Eigen::MatrixXd * knots_dist;
    Eigen::MatrixXd * knots_angle;

    const size_t getNumberOfDistanceKnots() const
    {
        return knots_dist->rows();
    }

    const size_t getNumberOfAngleKnots() const
    {
        return knots_angle->rows();
    }
};


class Observation
{
public:
    double intensity;
    double intensity_std;
    double distance;
    double angle;
    double eigen_ratio;
    size_t cloud_id;
    size_t core_id;

};

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

template <typename T>
T predict_intensities(const Observation &ob, const FixedModelPars &fixed_pars,  T const * const * parameters)
{
    typedef Eigen::Matrix<T,-1,1> MatrixType;
    typedef Eigen::Map<const MatrixType> MapType;

    const T * coeffs_dist = parameters[0];
    const T * coeffs_angle = parameters[1];

    MapType c_dist  (coeffs_dist, fixed_pars.getNumberOfDistanceKnots());
    MapType c_angle  (coeffs_angle, fixed_pars.getNumberOfAngleKnots());


    Eigen::Matrix<T, 1, 1> edist;
    edist << T(ob.distance);

    Eigen::Matrix<T, 1, 1> eang;
    eang<< T(ob.angle);


    T d_effect = compute_rbf<T>(c_dist, (fixed_pars.knots_dist)->template cast<T>(), T(fixed_pars.sigma_dist), edist );
    T a_effect = compute_rbf<T>(c_angle, (fixed_pars.knots_angle)->template cast<T>(), T(fixed_pars.sigma_angle), eang );


    T prediction = d_effect * a_effect;

    return prediction;
}

struct SmoothnessConstrain
{
    SmoothnessConstrain(const double alpha, FixedModelPars const * fixed_pars ): alpha_(alpha)
    {
        fixed_pars_ = fixed_pars;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {


        size_t d_s = fixed_pars_->getNumberOfDistanceKnots();
        size_t a_s = fixed_pars_->getNumberOfAngleKnots();




        for (int i = 0; i < d_s; ++i)
        {
            residual[i] = alpha_ * parameters[0][i];
        }

        for (int i = 0; i < a_s; ++i)
        {
            residual[i + d_s] = alpha_ * parameters[1][i];
        }






        return true;
    }

    size_t getNumberOfResiduals() const
    {
        return fixed_pars_->getNumberOfAngleKnots() + fixed_pars_->getNumberOfDistanceKnots();
    }

    double alpha_;
    const FixedModelPars * fixed_pars_;
};

struct ResidualFunctor
{
    ResidualFunctor(Observation * ob,
                    FixedModelPars * fixed_pars)


    {
        observation_ = ob;
        fixed_pars_ = fixed_pars;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {

        T prediction = predict_intensities(*observation_, *fixed_pars_, parameters);
        residual[0] = T(observation_->intensity) - prediction;


        return true;
    }

private:
    Observation * observation_;

    FixedModelPars * fixed_pars_;

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




std::vector<Observation> table2observations(const spc::EigenTable & table)
{
    std::vector<Observation> out(table.getNumberOfRows()) ;

    for (int i = 0; i < table.getNumberOfRows(); ++i)
    {
        Observation ob;

        ob.intensity = table.row(i)(table.getColumnId("intensity"));
        ob.intensity_std = table.row(i)(table.getColumnId("intensity_std"));
        ob.distance = table.row(i)(table.getColumnId("distance"));
        ob.angle = table.row(i)(table.getColumnId("angle"));
        ob.eigen_ratio = table.row(i)(table.getColumnId("eigen_ratio"));
        ob.cloud_id = table.row(i)(table.getColumnId("cloud_id"));
        ob.core_id = table.row(i)(table.getColumnId("core_id"));

        out.at(i) = ob;
    }

    return out;

}

int main (int argc, char ** argv)
{
    std::string datadb = "/home/luca/Desktop/calibration_db.spc";

    ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

    EigenTable::Ptr table = spcDynamicPointerCast<EigenTable> (o);

    table = table->getWithStrippedNANs({"distance", "intensity", "angle", "intensity_std"});

    std::cout << "Following columns found:\n" << std::endl;
    for (int i = 0; i < table->getNumberOfColumns(); ++i)
    {
        std::cout << table->getColumnName(i) << std::endl;
    }

    std::vector<Observation> obs = table2observations(*table);

    Eigen::VectorXf d = table->mat().col(table->getColumnId("distance"));
    Eigen::VectorXf a = table->mat().col(table->getColumnId("angle"));
    Eigen::VectorXf i = table->mat().col(table->getColumnId("intensity"));



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


    double dist_sigma = knots_distance->operator() (1) - knots_distance->operator ()(0);
    double angle_sigma = knots_angles->operator() (1) - knots_angles->operator ()(0);


    FixedModelPars * fixed_pars = new FixedModelPars;
    fixed_pars->knots_angle = knots_angles;
    fixed_pars->knots_dist = knots_distance;
    fixed_pars->sigma_dist = dist_sigma;
    fixed_pars->sigma_angle = angle_sigma;

    std::vector<double *> parameters(2);
    double dist_pars[n_dist_pars];
    double angle_pars[n_ang_pars];

    parameters[0] = &dist_pars[0];
    parameters[1] = &angle_pars[0];

    for (int i = 0; i < n_ang_pars; ++i)
        angle_pars[i] = 1.0;

    for (int i = 0; i < n_dist_pars; ++i)
        dist_pars[i] = 1.0;

    for (int j = 0 ; j < obs.size(); ++j)
    {

        Observation* ob = &obs.at(j);
        ResidualFunctor * f = new ResidualFunctor( ob, fixed_pars);

        ceres::DynamicAutoDiffCostFunction<ResidualFunctor, 4> * cost  = new ceres::DynamicAutoDiffCostFunction< ResidualFunctor, 4> ( f );

        cost->AddParameterBlock(n_dist_pars);
        cost->AddParameterBlock(n_ang_pars);
        cost->SetNumResiduals(1);




        problem.AddResidualBlock(cost,
                                 //                                 new ceres::ScaledLoss(NULL, 1/(i_std(j)*i_std(j)), ceres::TAKE_OWNERSHIP),
                                 NULL,
                                 //                                 HuberLoss(0.00001),
                                 parameters
                                 );

    }

    SmoothnessConstrain * smoothness = new SmoothnessConstrain(0.05 * obs.size(), fixed_pars);
    ceres::DynamicAutoDiffCostFunction<SmoothnessConstrain, 4> * cost2  = new ceres::DynamicAutoDiffCostFunction< SmoothnessConstrain, 4> ( smoothness );

    cost2->AddParameterBlock(n_dist_pars);
    cost2->AddParameterBlock(n_ang_pars);
    cost2->SetNumResiduals(smoothness->getNumberOfResiduals());

    problem.AddResidualBlock(cost2, NULL, parameters);


    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.function_tolerance = 1e-6;
    options.gradient_tolerance = 1e-6;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";



    std::cout << "dist pars:" << std::endl;
    for (int i =0; i <n_dist_pars ; ++i)
    {
        std::cout << dist_pars[i] << std::endl;
    }

    std::cout << "angle pars:" << std::endl;
    for (int i =0; i <n_ang_pars; ++i)
    {
        std::cout << angle_pars[i] << std::endl;
    }







    /////////////////////////////////////////////// recomputing

    Eigen::VectorXd predicted(obs.size());

    for (int j = 0; j < obs.size(); ++j)
    {
        predicted(j) = predict_intensities(obs.at(j), *fixed_pars,  &parameters[0]);
    }

    EigenTable::Ptr out (new EigenTable);
    out->addNewComponent("distance", 1);
    out->addNewComponent("angle", 1);
    out->addNewComponent("intensity", 1);
    out->addNewComponent("pred_intensity", 1);


    out->resize(i.rows());

    out->column("distance") = d;
    out->column("angle") = a;
    out->column("intensity") = i.cast<float>();
    out->column("pred_intensity") = predicted.cast<float>();


    spc::io::AsciiEigenTableWriter w;
    w.setInput(out);
    w.setOutputFilename("/home/luca/test.txt");
    w.setWriteHeaders(true);
    w.write();

    return 1;
}
