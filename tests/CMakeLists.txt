project(tests)
link_directories(${CC_BUILD_DIR}/qCC/plugins/qPCL/PclUtils/)

add_executable(testPlotter testPlotter.cpp)
target_link_libraries(testPlotter PclUtils ${SPC_LIBRARIES} qGEOTools qGEOVis ${QT_LIBRARIES} qGEO)
