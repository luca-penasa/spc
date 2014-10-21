#include <spc/methods/common.hpp>

#include <flann/flann.hpp>

namespace spc
{



//// INST
template std::vector<float> subdivideRange
    <float>(const float start, const float end, const float step);
template std::vector<double> subdivideRange
    <double>(const double start, const double end, const double step);

} // end namespace
