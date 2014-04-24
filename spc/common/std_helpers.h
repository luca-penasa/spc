#ifndef STD_HELPERS_H
#define STD_HELPERS_H

#include <vector>

namespace spc
{
////////////////// THES THINGS ARE ~ DEPRECATED ///////////////////

/// SCALAR RETURNING

template <typename ScalarT>
ScalarT get_min(const std::vector<ScalarT> &invect);

template <typename ScalarT>
ScalarT get_max(const std::vector<ScalarT> &invect);

template <typename ScalarT>
ScalarT get_sum(const std::vector<ScalarT> &input);



//// VECTOR RETURNING
template <typename ScalarT>
std::vector<ScalarT>
get_difference(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2);

template <typename ScalarT>
std::vector<ScalarT>
get_difference(const std::vector<ScalarT> &input, const ScalarT &value);

template <typename ScalarT>
 std::vector<ScalarT>get_squared(const std::vector<ScalarT> &input)  ;

template <typename ScalarT>
std::vector<ScalarT> get_product(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2);

//! says if the element is present in input vector
template <typename T>
bool element_exists (const std::vector<T> &input, const T &element);


}//end nspace

#endif

