#ifndef PARAMETERSHOLDER_H
#define PARAMETERSHOLDER_H

#include <vector>
#include <cstddef>
#include <glog/logging.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
namespace spc
{


class ParameterBlock
{
public:

    //! A matrix-like block of parameters
    ParameterBlock(size_t n_rows,
                      std::string block_name,
                      size_t n_columns = 1,
                      double init_value = 1,
                      bool active=true):
        is_enabled_(active),
        block_name_(block_name)

    {

        data_ = Eigen::MatrixXd(n_rows, n_columns);
        data_.setConstant(init_value);

    }

    void disable()
    {
        is_enabled_ = false;
//        block_id_in_problem_ = -1;
    }

    void enable()
    {
        is_enabled_ = true;
    }

    bool isEnabled() const
    {
        return is_enabled_;
    }


    Eigen::MatrixXd &getData()
    {
        return data_;
    }

    double * getDataPtr()
    {
        return data_.data();
    }

    size_t getBlockSize() const
    {
        return data_.size();
    }

    size_t getNColumns() const
    {
        return data_.cols();
    }

    size_t getNRows() const
    {
        return data_.rows();
    }

    std::string getBlockName() const
    {
        return block_name_;
    }



//    void setBlockIdInProblem(const int &id)
//    {
//        block_id_in_problem_ = id;
//    }

//    int getBlockIdInProblem() const
//    {
//        return block_id_in_problem_;
//    }

    void printMe() const
    {

        std::cout << "BLOCK: "<< block_name_ << std::endl;
//        std::cout << "Overall ID " << block_id_in_problem_ << std::endl;
        std::cout << "Data\n" << data_ << std::endl;
        if (isEnabled())
        {
            std::cout << "I am enabled for optimization!" << std::endl;
        }
        else
            std::cout << "I am NOT enabled for optimization!" << std::endl;

    }
protected:
    Eigen::Matrix<double , -1, -1> data_;

    std::string block_name_;

//    int block_id_in_problem_ = -1;
    bool is_enabled_ = true;


};




}//end nspace

#endif // PARAMETERSHOLDER_H
