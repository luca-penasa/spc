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


class MetaBlock
{
public:

    //! A matrix-like block of parameters
    MetaBlock(size_t n_rows,
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
        block_id_in_problem_ = -1;
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



    void setBlockIdInProblem(const int &id)
    {
        block_id_in_problem_ = id;
    }

    int getBlockIdInProblem() const
    {
        return block_id_in_problem_;
    }

    void printMe() const
    {

        std::cout << "BLOCK: "<< block_name_ << std::endl;
        std::cout << "Overall ID " << block_id_in_problem_ << std::endl;
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

    int block_id_in_problem_ = -1;
    bool is_enabled_ = true;


};



//! containes both active and inactive blocks of parameters
//! which are parameters that are or not optimized but are used
//! by the residual functor.
class ParametersDescriptor
{

public:

    void printBlocksResumee() const
    {
        size_t n = 0;
        for (MetaBlock * b: blocks_)
        {

            std::cout << " -----------> BLOCK N " << n++ << std::endl;
            b->printMe();
        }
    }

    size_t getNumberOfBlocks() const
    {
        return blocks_.size();
    }

    void pushBack(MetaBlock * block)
    {
        CHECK (name_to_block_id_.find(block->getBlockName()) == name_to_block_id_.end()) << "A block with this name yet exists! " << block->getBlockName();

        blocks_.push_back(block);
        name_to_block_id_[block->getBlockName()] = blocks_.size() - 1;

        std::cout << "pushed back a new block " << block->getBlockName() << " with id " << name_to_block_id_[block->getBlockName()] << " of " << blocks_.size() << std::endl;

        if (block->isEnabled()) // we give him an overall id
        {

            std::cout << "The block is enabled! we assign an overall id of: " << this->getNumberOfActiveBlocks() -1 << std::endl;
            block->setBlockIdInProblem(this->getNumberOfActiveBlocks() -1);

            std::cout << "its id as active block is " << block->getBlockIdInProblem() << std::endl;
        }

        updateActiveBlocksPointers();
    }

    size_t getNumberOfActiveBlocks()
    {
        size_t n = 0;
        for (MetaBlock * b: blocks_)
        {
            if (b->isEnabled())
            {
                n += 1;
            }

        }
        std::cout << "there are currently " << n << " active blocks" << std::endl;
        return n;
    }

    void pushBack(std::vector<MetaBlock *> blocks)
    {
        for (MetaBlock *b: blocks)
        {
            this->pushBack(b);
        }

    }



    MetaBlock * getBlockFromName(const std::string name) const
    {
        size_t id = name_to_block_id_.at(name);

        return blocks_.at(id);
    }

    void updateActiveBlocksPointers()
    {
        active_blocks_ptrs_.clear();

        for (MetaBlock * b: blocks_)
        {
            active_blocks_ptrs_.push_back(b->getDataPtr());
        }
    }

    double const * const * getAllActiveParsPtr() const
    {
        return &active_blocks_ptrs_[0];
    }

protected:
    std::vector<MetaBlock *> blocks_;

    std::map<std::string, size_t> name_to_block_id_;

    std::vector<double *> active_blocks_ptrs_;



};

}//end nspace

#endif // PARAMETERSHOLDER_H
