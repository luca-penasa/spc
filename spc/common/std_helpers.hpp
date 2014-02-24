#ifndef SPC_STD_HELPERS_IMPL_H
#define SPC_STD_HELPERS_IMPL_H

#include "std_helpers.h"

namespace spc
{

template <typename ScalarT>
ScalarT
get_min(const std::vector<ScalarT> &invect)
{
    int n = invect.size();
    ScalarT value = invect[0];
    for (int i = 1; i < n; ++i)
    {
        if (invect[i] < value)
            value = invect[i];
    }
    return value;
}


template <typename ScalarT>
ScalarT
get_max(const std::vector<ScalarT> &invect)
{
    int n = invect.size();
    ScalarT value = invect[0];
    for (int i = 1; i < n; ++i)
    {
        if (invect[i] > value)
            value = invect[i];
    }
    return value;
}


template <typename ScalarT>
ScalarT
get_sum(const std::vector<ScalarT> &input)
{
    ScalarT sum = 0;
    for (auto &elem : input)
      sum += elem;

    return sum;
}

template <typename ScalarT>
ScalarT
get_mean(const std::vector<ScalarT> &input)
{
    ScalarT sum = get_sum(input);
    return sum / input.size() - 1;

}

template <typename ScalarT>
std::vector<ScalarT>
get_difference(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2)
{
    std::vector<ScalarT> difference;
    difference.resize(input1.size());
    for (int i = 0; i < input1.size(); ++i)
        difference[i] = input1[i] - input2[i];

    return difference;
}


template <typename ScalarT>
std::vector<ScalarT>
get_difference(const std::vector<ScalarT> &input1, const ScalarT value)
{
    std::vector<ScalarT> difference;
    difference.resize(input1.size());

    for (int i = 0; i < input1.size(); ++i)
        difference[i] = input1[i] - value;

    return difference;
}

template<typename ScalarT>
std::vector<ScalarT>
get_squared(const std::vector<ScalarT> &input)
{
    std::vector<ScalarT> squared;
    squared.resize(input.size());
    for (int i = 0; i < input.size(); ++i)
        squared[i] = input[i]*input[i];

    return squared;

}

template<typename ScalarT>
std::vector<ScalarT>
get_product(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2)
{
    std::vector<ScalarT> product;
    product.resize(input1.size());
    for (int i = 0; i < input1.size(); ++i)
        product[i] = input1[i]*input2[i];

    return product;
}


} //end nspace
#endif


