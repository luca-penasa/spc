#ifndef SPC_COMMON_HPP
#define SPC_COMMON_HPP

#include <spc/methods/common.h>
#include <spc/methods/InterpolatorRBF.h>

namespace spc {

template <class nType>
std::vector<nType> subdivideRange(const nType start, const nType end,
                                  const nType step)
{
    nType range = std::abs(end - start);
    int n_steps = ceil(range / step) + 1;

    std::vector<nType> result;
    result.resize(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        result[i] = start + i * step;
    }
    return result;
}

template <typename nType>
void fill_nan_rbf(std::vector<nType> &x, std::vector<nType> &y)
{
    // get nans ids
    std::vector<int> nans_id = get_nans_id(y);

    std::vector<nType> x_stripped;
    std::vector<nType> y_stripped;

    for (auto &elem : nans_id) {
        // remove this ids from the original dataset
        x_stripped = erease_ids(x, nans_id);
        y_stripped = erease_ids(y, nans_id);
    }
    // now create a rbf interpolator
    spc::InterpolatorRBF<nType, 1> rbf;
    rbf.setPoints(x_stripped);
    rbf.setInputValues(y_stripped);
    rbf.updateAll();

    for (auto &elem : nans_id) {
        int this_id = elem;
        nType this_x_position = x[this_id];
        Eigen::Matrix<nType, Eigen::Dynamic, 1> x_point;
        x_point.resize(1);
        x_point[0] = this_x_position;
        y[this_id] = rbf.evaluateRbf(x_point);
    }

    // now for each nan evaluate the rbf
}

}


#endif // COMMON_HPP
