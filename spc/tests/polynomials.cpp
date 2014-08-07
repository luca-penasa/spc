#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

//! and evaluable object has input and output.
//! both input and output are matrix, each in/out variable is in a different
//column
//! getInFieldsnames and getOutFieldsNames() permits to suggest naming for
//input/output channels
class EigenEvaluableObject
{
public:
    virtual size_t getInDim() const = 0;
    virtual size_t getOutDim() const = 0;

    virtual Eigen::VectorXf operator()(const Eigen::VectorXf &x) const = 0;

    virtual std::vector<std::string> getInFieldsEvalNames() const = 0;

    virtual std::vector<std::string> getOutFieldsEvalNames() const = 0;
};

//! a fittable object must also be evaluable
class FittableObject : public EigenEvaluableObject
{
public:
    enum FitIface {
        LSQ,
        NONE
    };

    /** get which kind of optimization is used for the fit
     *  for this object
     */
    virtual FitIface getMyFitInterface() const = 0;

    /** fit the object to the dataset provided
     */
    virtual int fitTo(const Eigen::MatrixXf &data) = 0;

    //! zero for a function that hasnt be fitted.
    size_t getNumberOfParameters() const
    {
        return parameters_.size();
    }

    Eigen::MatrixXf getParameters() const
    {
        return parameters_;
    }

protected:
    Eigen::MatrixXf parameters_;
};

//! an object that can be fitted using linear least squares.
class FittableObjectLSQ : public FittableObject
{
    // FittableObjectIFace interface
public:
    virtual FitIface getMyFitInterface() const
    {
        return LSQ;
    }
    /**
       * return a vector of the evaluated monomials, ( without multiplication
     * with coefficient)
       * this method completely determine the shape of the function to fit to
     * data
     */
    virtual Eigen::VectorXf evaluateMonomialsVector(const Eigen::VectorXf
                                                    &vars) const = 0;

    virtual Eigen::MatrixXf getAMatrix(const Eigen::MatrixXf &x) const
    {
        Eigen::MatrixXf out;
        out.resize(x.rows(), x.cols());

        std::cout << x << std::endl;

        for (int i = 0; i < x.rows(); ++i) {
            std::cout << this->evaluateMonomialsVector(x.row(i)) << std::endl;
            out.row(i) = this->evaluateMonomialsVector(x.row(i));
        }

        return out;
    }

    //! this will fit the coefficients using considering data as composed by:
    //! [x1,x2,x3, .... y] with xN independent variables and y dependent
    virtual int fitTo(const Eigen::MatrixXf &data)
    {
        Eigen::MatrixXf x = data.block(0, 0, data.rows(), data.cols() - 1);
        Eigen::MatrixXf y = data.col(data.cols() - 1);

        Eigen::MatrixXf A = this->getAMatrix(x);
        parameters_ = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(y);
        return 1;
    }

public:
    virtual size_t getOutDim() const
    {
        return 1;
    }
    virtual Eigen::VectorXf operator()(const Eigen::VectorXf &x) const

    {
        Eigen::VectorXf mon = this->evaluateMonomialsVector(x);
        Eigen::VectorXf res = mon.array() * parameters_.array();
        Eigen::VectorXf out;
        out << res.sum();
        return out;
    }
};

class CalibrationModelPfeiferHofle : public FittableObjectLSQ
{

    // FittableObjectLSQ interface
public:
    virtual VectorXf evaluateMonomialsVector(const VectorXf &vars) const
    {
        float w, r;
        w = cos(vars(0) / 180 * M_PI);
        r = vars(1);

        VectorXf out(16);
        out << pow(w, 3) * pow(r, 3);
        out << pow(w, 3) * pow(r, 2);
        out << pow(w, 3) * pow(r, 1);
        out << pow(w, 3);
        out << pow(w, 2) * pow(r, 3);
        out << pow(w, 2) * pow(r, 2);
        out << pow(w, 2) * pow(r, 1);
        out << pow(w, 2);
        out << pow(w, 1) * pow(r, 3);
        out << pow(w, 1) * pow(r, 2);
        out << pow(w, 1) * pow(r, 1);
        out << pow(w, 1);
        out << pow(r, 3);
        out << pow(r, 2);
        out << pow(r, 1);
        out << 1;

        return out;
    }

    // EigenEvaluableObject interface
public:
    virtual size_t getInDim() const
    {
        return 2;
    }
    virtual std::vector<string> getInFieldsEvalNames() const
    {
        return std::vector<string> {"distance", "angle"};
    }

    virtual std::vector<std::string> getOutFieldsEvalNames() const
    {
        return std::vector<string> {"intensity"};

    }


};

int main()
{
    CalibrationModelPfeiferHofle model;

    Eigen::MatrixXf a = Eigen::MatrixXf::Random(100, 4);
    a.col(0).array() *=100;
    a.col(1).array() *=10;

    model.fitTo(Eigen::MatrixXf::Random(100, 4));

    std::cout << model.getParameters() << std::endl;

    return 1;
}
