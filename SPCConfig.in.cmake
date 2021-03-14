
@PACKAGE_INIT@

include(CMakeFindDependencyMacro)

set(SPC_HAS_PCL "@SPC_HAS_PCL@")
set(SPC_MODULES @SPC_MODULES@)

if (SPC_HAS_PCL)
    find_dependency(PCL)
else()
    find_dependency(Eigen3)
    find_dependency(Boost COMPONENTS regex filesystem)
    find_dependency(OpenMP)
endif()

find_dependency(OpenMP)

message("finding comps ${SPC_FIND_COMPONENTS}")
foreach(_comp ${SPC_MODULES})


    #//  if (NOT _comp IN_LIST _supported_components)
    #//    set(MathFunctions_FOUND False)
    #//    set(MathFunctions_NOT_FOUND_MESSAGE "Unsupported component: ${_comp}")
    #//  endif()
    include("${CMAKE_CURRENT_LIST_DIR}/${_comp}-export.cmake")
endforeach()
