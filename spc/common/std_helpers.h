#ifndef STD_HELPERS_H
#define STD_HELPERS_H


//FROM PCL

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/impl/point_types.hpp>

//FROM LIDARLIB
//#include "point_types.h"

#include <vector>




/// SCALAR RETURNING

template <typename ScalarT>
auto
get_min(const std::vector<ScalarT> &invect) -> ScalarT;

template <typename ScalarT>
auto
get_max(const std::vector<ScalarT> &invect) -> ScalarT;

template <typename ScalarT>
auto
get_sum(const std::vector<ScalarT> &input) -> ScalarT;



//// VECTOR RETURNING
template <typename ScalarT>
auto
get_difference(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2) -> std::vector<ScalarT>;

template <typename ScalarT>
auto
get_difference(const std::vector<ScalarT> &input, const ScalarT &value) -> std::vector<ScalarT>;

template <typename ScalarT>
auto
get_squared(const std::vector<ScalarT> &input) -> std::vector<ScalarT>;

template <typename ScalarT>
auto
get_product(const std::vector<ScalarT> &input1, const std::vector<ScalarT> &input2) -> std::vector<ScalarT>;
#endif

