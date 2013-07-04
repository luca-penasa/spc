set(MY_TOOLS_PREFIX "sb_")


# include_directories(${LIDARLIB_SOURCE_DIR})

find_package( OpenCV REQUIRED )


file(GLOB SOURCES *.cpp)
file(GLOB HEADERS *.h)
set(files "${SOURCES}" "${HEADERS}")


add_executable(${MY_TOOLS_PREFIX}compute_ties ${files})
install(TARGETS ${MY_TOOLS_PREFIX}compute_ties DESTINATION bin)


target_link_libraries(${MY_TOOLS_PREFIX}compute_ties ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES} flann  ${OpenCV_LIBS})



