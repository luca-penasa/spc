#ifndef INTENSITYMODELLINEAR_H
#define INTENSITYMODELLINEAR_H
#include <spc/ceres_calibration/IntensityModelBase.h>

namespace spc
{

// bivariate polynomial
// the sum of the retrun matrix is the value
Eigen::MatrixXf polyTerms(const size_t n, const size_t m, Eigen::Matrix<float,2,1> values);

class IntensityModelLinear: public IntensityModelBase
{
public:
    spcTypedefSharedPtrs(IntensityModelLinear)


    virtual Eigen::VectorXf getAMatrixRow(const Eigen::VectorXf &point) const = 0;

};


class IntensityModelRBF: public IntensityModelLinear
{
public:
    spcTypedefSharedPtrs(IntensityModelRBF)

    IntensityModelRBF();

    // IntensityModelBase interface
public:
    virtual float getPredictedIntensity(const Eigen::VectorXf &point) const
    {
        // compute the RBF value
//        return weights.cwiseProduct(coeffs_ ).sum();
    }




public:
    virtual Eigen::VectorXf getAMatrixRow(const Eigen::VectorXf &point) const
    {

        Eigen::VectorXf pol = getPolynomialPart(point);
        Eigen::VectorXf rbf  = getRBFPart(point);
        Eigen::VectorXf out;
        out.resize(pol.rows() + rbf.rows());

        out << rbf;
        out << pol;

        return out;
    }

    virtual std::vector<FieldToFieldNameMapper::FIELD_ENUM> getRequiredFields() const
    {
        return fields_needed_;
    }

protected:
    Eigen::VectorXf getRBFPart(const Eigen::VectorXf &point) const
    {
        Eigen::VectorXf p = scales_.array() * point.array();

        // compute squared distances of point from nodes
        Eigen::MatrixXf diff  = nodes_.rowwise() - p.transpose();
        Eigen::VectorXf sq_dist =  diff.rowwise().squaredNorm();

        // transform these sq_dist in weights
        Eigen::VectorXf weights(sq_dist.rows());

        for (int i = 0; i < sq_dist.rows(); ++i)
            weights(i)  = exp(- sq_dist(i) / (sigma_squared_)  );

        return weights;
    }

    Eigen::VectorXf getPolynomialPart(const Eigen::VectorXf &point) const
    {
        Eigen::VectorXf p = point;
        p(1) = cos( p(1) / M_PI * 180 ); // polynomial part on the cos of the angle.
        return polyTerms(2,2, point);
    }

protected:
    Eigen::MatrixXf nodes_;
    Eigen::VectorXf coeffs_;


    Eigen::VectorXf scales_;

    float sigma_squared_;

    std::vector<FieldToFieldNameMapper::FIELD_ENUM> fields_needed_ = {FieldToFieldNameMapper::DISTANCE,
                                                                     FieldToFieldNameMapper::ANGLE};




};
}//end nspce

#endif // INTENSITYMODELLINEAR_H
