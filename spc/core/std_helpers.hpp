#ifndef SPC_STD_HELPERS_IMPL_H
#define SPC_STD_HELPERS_IMPL_H

#include "std_helpers.h"
#include <algorithm>
namespace spc
{

template <typename T>
bool element_exists (const std::vector<T> &input, const T &element)
{
    return std::find(input.begin(), input.end(), element) != input.end();
}


} //end nspace
#endif


