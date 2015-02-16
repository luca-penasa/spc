





find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/)
# include_directories(${LIDARLIB_SOURCE_DIR})

spc_add_executable(example_attitude example_attitude.cpp)
target_link_libraries(example_attitude ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


spc_add_executable(example_time_series_generator example_time_series_generator.cpp)
target_link_libraries(example_time_series_generator ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


