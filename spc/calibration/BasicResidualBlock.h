#ifndef BASICRESIDUALBLOCK_H
#define BASICRESIDUALBLOCK_H

#include <ceres/ceres.h>
#include <spc/calibration/ParametersBlock.h>
#include <spc/calibration/HelperMethods.h>

namespace spc
{


class BasicResidualBlock
{
public:


    virtual ceres::CostFunction * getMyCost()  = 0;


    std::vector<ParameterBlock *> getMyBlocks()
    {
        return blocks_;
    }

    std::vector<double *> getMyActiveParameters() const
    {
        std::vector<double *> out;
        for (ParameterBlock * b: blocks_)
        {
            if (b->isEnabled())
                out.push_back(b->getDataPtr());
        }

//        std::cout <<"found " << out.size() << " active pars" << std::endl;
        return out;
    }

    void updateNameToBlock()
    {
        name_to_block_.clear();
        size_t n = 0;
        for (ParameterBlock * b: blocks_)
        {
                std::string name = b->getBlockName();
                name_to_block_[name] = b;

                if (b->isEnabled())
                    id_in_active_pars_[b] = n++;
        }
    }

    template <typename T>
    Eigen::Matrix<T, -1,-1> remapFromPointerAndName(const T * const * data, const std::string par_name) const
    {
        ParameterBlock  *b = name_to_block_.at(par_name);

        return remapFromPointerAndBlock(data, b);
    }

    template <typename T>
    Eigen::Matrix<T, -1,-1> remapFromPointerAndBlock(const T * const * data, ParameterBlock *b) const
    {
        if (b->isEnabled())
        {
            size_t id_in_pars = id_in_active_pars_.at(b);
            Eigen::Map<const Eigen::Matrix<T, -1,-1>> mat = Eigen::Map<const Eigen::Matrix<T, -1, -1>> (data[id_in_pars], b->getNRows(), b->getNColumns());

//            std::cout << "remapped parameters for block " << b->getBlockName() <<std::endl;
//            printMatrix(mat);
            return mat;
        }
        else
            return b->getData().cast<T>();


    }


    std::vector<ParameterBlock *> getParametersBlocksByName(const std::vector<std::string> & block_names)
    {
        std::vector<ParameterBlock *> blocks;
        for (std::string b_name: block_names)
        {
            blocks.push_back(name_to_block_.at(b_name));
        }

        return blocks;
    }


protected:
    std::vector<ParameterBlock *> blocks_;
    std::map<std::string, ParameterBlock *> name_to_block_;
    std::map<ParameterBlock *, size_t> id_in_active_pars_;

};

}// end nspace

#endif // BASICRESIDUALBLOCK_H
