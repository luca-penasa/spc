#ifndef STD_HELPERS_IMPL_H
#define STD_HELPERS_IMPL_H

#include "std_helpers.h"


template <typename ScalarT>
auto
get_min(const std::vector<ScalarT> &invect) -> ScalarT
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
auto
get_max(const std::vector<ScalarT> &invect) -> ScalarT
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
auto
get_sum(const std::vector<ScalarT> &input) -> ScalarT
{
    ScalarT sum = 0;
    for (int i = 0; i < input.size(); ++i)
        sum += input[i];

    return sum;
}

template <typename ScalarT>
auto
get_mean(const std::vector<ScalarT> &input) -> ScalarT
{
    ScalarT sum = get_sum(input);
    return sum / input.size() - 1;

}

template <typename ScalarT>
auto
get_difference(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2) -> std::vector<ScalarT>
{
    std::vector<ScalarT> difference;
    difference.resize(input1.size());
    for (int i = 0; i < input1.size(); ++i)
        difference[i] = input1[i] - input2[i];

    return difference;
}


template <typename ScalarT>
auto
get_difference(const std::vector<ScalarT> &input1, const ScalarT value) -> std::vector<ScalarT>
{
    std::vector<ScalarT> difference;
    difference.resize(input1.size());

    for (int i = 0; i < input1.size(); ++i)
        difference[i] = input1[i] - value;

    return difference;
}

template<typename ScalarT>
auto
get_squared(const std::vector<ScalarT> &input) -> std::vector<ScalarT>
{
    std::vector<ScalarT> squared;
    squared.resize(input.size());
    for (int i = 0; i < input.size(); ++i)
        squared[i] = input[i]*input[i];

    return squared;

}

template<typename ScalarT>
auto
get_product(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2) -> std::vector<ScalarT>
{
    std::vector<ScalarT> product;
    product.resize(input1.size());
    for (int i = 0; i < input1.size(); ++i)
        product[i] = input1[i]*input2[i];

    return product;
}

#endif


