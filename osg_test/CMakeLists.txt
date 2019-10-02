find_package(SPC)
include_directories(${SPC_INCLUDE_DIRS})

add_definitions(${SPC_DEFINITIONS})


FIND_PACKAGE( OpenSceneGraph REQUIRED osgViewer)
message("libs ${OPENSCENEGRAPH_LIBRARIES}")
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

add_executable(test1 test1.cpp )
target_link_libraries(test1 ${SPC_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES})
