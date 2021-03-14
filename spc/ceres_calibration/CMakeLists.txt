
find_package(Ceres REQUIRED)
set(libs ${CERES_LIBRARIES})
spc_add_library(calibration)
#include_directories(${CERES_INCLUDE_DIRS})
#target_link_libraries(spc_ceres_calibration ${CERES_LIBRARIES})
#ceres libraries must be linked
