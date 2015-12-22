#include "IntensityModelLinear.h"
namespace spc
{

Eigen::MatrixXf polyTerms(const size_t n, const size_t m, Eigen::Matrix<float, 2, 1> values)
{
    Eigen::Matrix<float, -1, -1> out;
    out.resize(n, m);

    for (int i = 0 ; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            out(i,j) =  pow(values(0), i) * pow(values(1), j);
        }
    }
    return out;
}


}//end nspace
