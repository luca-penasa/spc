#pragma once
#ifndef FUNCTIONBASE_H
#define FUNCTIONBASE_H

#include <Eigen/Dense>
#include <pcl/console/print.h>
#include <spc/elements/ElementBase.h>
namespace spc
{

class EigenFunctionBase : public ElementBase
{
public:
    SPC_OBJECT(EigenFunctionBase)
    EXPOSE_TYPE

    EigenFunctionBase()
    {
    }

    EigenFunctionBase(const size_t &input_s, const size_t &output_s)
    {
        input_size_ = input_s;
        output_size_ = output_s;
    }

    virtual Eigen::VectorXf At(const Eigen::VectorXf &v) = 0;

    virtual Eigen::MatrixXf At(const Eigen::MatrixXf &v)
    {
          Eigen::MatrixXf out;
          out.resize(v.rows(), this->getOutputSize());

          for (int i = 0; i < v.rows(); ++i)
          {
            Eigen::VectorXf row = v.row(i);
            out.row(i) = this->At(row);
          }

          return out;
    }

    spcGetMacro(InputSize, input_size_, size_t)
    spcGetMacro(OutputSize, output_size_, size_t)

protected:
    size_t input_size_;
    size_t output_size_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(input_size_),
           CEREAL_NVP(output_size_));
    }
};



} // end nspace

#endif // FUNCTIONBASE_H
