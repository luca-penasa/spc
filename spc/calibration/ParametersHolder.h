#ifndef PARAMETERSHOLDER_H
#define PARAMETERSHOLDER_H

#include <vector>
#include <cstddef>
namespace spc
{



class ParametersHolder
{
    typedef std::vector<double> ParBlockT;
public:
    ParametersHolder();

    std::vector<ParBlockT> parameters;


    std::vector<double *> getAsVectorOfPointers() const
    {
        std::vector<double *> out;
        for (ParBlockT p: parameters)
            out.push_back(&p[0]);

        return out;
    }

    size_t getNumberOfParametersInBlock(size_t block_id) const
    {
        return parameters.at(block_id).size();
    }


    std::vector<size_t> getSizesOfBlocks() const
    {
        std::vector<size_t> out;
        for (ParBlockT p: parameters)
            out.push_back(p.size());

        return out;
    }

};

}//end nspace

#endif // PARAMETERSHOLDER_H
