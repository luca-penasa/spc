#ifndef BASICRESIDUALBLOCK_H
#define BASICRESIDUALBLOCK_H

#include <ceres/ceres.h>
#include <spc/calibration/ParametersHolder.h>

namespace spc
{


class BasicResidualBlock
{
public:
//    BasicResidualBlock()
//    {
//    }

    virtual ceres::CostFunction * getMyCost()  = 0;

    virtual void initMyBlocks() = 0;

    std::vector<MetaBlock *> getMyBlocks()
    {
        return blocks_;
    }

    std::vector<double *> getMyActiveParameters() const
    {
        std::vector<double *> out;
        for (MetaBlock * b: blocks_)
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
        for (MetaBlock * b: blocks_)
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
        MetaBlock  *b = name_to_block_.at(par_name);

        return remapFromPointerAndBlock(data, b);
    }

    template <typename T>
    Eigen::Matrix<T, -1,-1> remapFromPointerAndBlock(const T * const * data, MetaBlock *b) const
    {
        if (b->isEnabled())
        {
            size_t id_in_pars = id_in_active_pars_.at(b);
            return Eigen::Map<const Eigen::Matrix<T, -1, -1>> (data[id_in_pars], b->getNRows(), b->getNColumns());
        }
        else
            return b->getData().cast<T>();
    }



protected:
    std::vector<MetaBlock *> blocks_;

//    ParametersDescriptor * parameter_descriptor_;

    std::map<std::string, MetaBlock *> name_to_block_;
    std::map<MetaBlock *, size_t> id_in_active_pars_;

};

}// end nspace

#endif // BASICRESIDUALBLOCK_H
