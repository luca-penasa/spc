#ifndef SPC_COMMON_INCLUDES_H
#define SPC_COMMON_INCLUDES_H


//STD LIB
#include <vector>
#include <algorithm>
#include <numeric>
#include <map>
#include <iostream>



//BOOST STUFF
#include <boost/smart_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

// EIGEN
#include <Eigen/Core>

// PCL
#include <pcl/console/print.h>
namespace  spc {

#ifndef NAN
    #define NAN  std::numeric_limits<float>::quiet_NaN();
#endif

}

#endif // SPC_COMMON_INCLUDES_H
