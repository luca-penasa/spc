#ifndef HELPERMETHODS_H
#define HELPERMETHODS_H


#include <algorithm>
//#include <Eigen/Eigen>

namespace spc {
//using Eigen::Matrix;

//! an unique that should work with eigen types
template <typename ObjT>
ObjT unique(const ObjT& b)
{
    ObjT tmp = b;


    std::sort(tmp.data(), tmp.data() + tmp.size());
    auto last = std::unique(tmp.data(), tmp.data() + tmp.size());

    size_t n_elements = last - tmp.data();

    tmp.conservativeResize(n_elements);

    return tmp;
}


}// end nspace
#endif // HELPERMETHODS_H
