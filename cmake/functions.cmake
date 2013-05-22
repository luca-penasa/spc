macro(spc_compile_and_link name files libs)
#    message("${name}")
    if(BUILD_SPC_SHARED)
            add_library (${name} SHARED ${files})           
    else()
            add_library (${name} STATIC ${files})           
    endif()
    target_link_libraries(${name} ${libs})
    spc_install_target_library_libs (${name})

    #update the variable keeping the libarires

    set(SPC_LIBRARIES ${SPC_LIBRARIES} ${name} CACHE INTERNAL "SPC libs")
endmacro()


function(spc_add_library libname dependence)

    file(GLOB @libname@_SOURCES *.cpp)
    file(GLOB @libname@_HEADERS *.h)
    file(GLOB @libname@_IMPLS *.hpp)


    set(sources ${@libname@_SOURCES} ${@libname@_INCLUDES} ${@libname@_IMPLS})
    set(libs ${dependence} ${SCP_LIBRARIES})
    spc_compile_and_link(spc_${libname} "${sources}" "${libs}")

    set(all_heads ${@libname@_HEADERS} ${@libname@_IMPLS})
    spc_install_target_library_headers (${libname} "${all_heads}")


endfunction()

macro(spc_library_needs_pcl)
    find_package(PCL 1.3 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    set(additional_libs ${additional_libs} ${PCL_LIBRARIES})
endmacro()

macro(spc_add_executable name codefiles)
    add_executable(${name} ${codefiles})
    install (TARGETS ${name} DESTINATION bin)
endmacro()

macro(spc_install_target_library_headers libname headers)
    install(FILES ${headers} DESTINATION include/spc/${libname})
endmacro()

macro(spc_install_target_library_libs libname)
    install(TARGETS "${libname}" DESTINATION lib)
endmacro()
