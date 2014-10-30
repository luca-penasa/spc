#ifndef STD_HELPERS_H
#define STD_HELPERS_H

#include <vector>

namespace spc
{


//! says if the element is present in input vector
template <typename T>
bool element_exists(const std::vector<T> &input, const T &element);

} // end nspace

#endif
