#compiles and link a spc_lib
macro(spc_compile_and_link name files libs)
    if(SPC_BUILD_SHARED)
        add_library(${name} SHARED ${files})
    else()
        add_library(${name} STATIC ${files})
    endif()

    target_link_libraries(${name} ${libs})

    #    spc_install_target_library_libs(${name})

    #update the variable keeping the libarires
    set(SPC_LIBRARIES ${SPC_LIBRARIES} ${name} CACHE INTERNAL "SPC libs")
    set(SPC_BUILD_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "SPC include dirs at build time")
endmacro()

# this macro add a library to SPC
macro(spc_add_library)
    get_filename_component(libname ${CMAKE_CURRENT_SOURCE_DIR}  NAME)

    # Auto add all subdirectories
    file(GLOB subdirectories *)

    set(LIB_SOURCES "")
    set(LIB_HEADERS "")
    set(LIB_IMPLS "")

    foreach(dir ${subdirectories})
        if(IS_DIRECTORY ${dir})
            message("found dir " ${dir})
            get_filename_component(subdirname ${dir}  NAME)
            if(SPC_ENABLE_INSTALL)
                install(DIRECTORY ${dir} DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/${libname}"
                    FILES_MATCHING PATTERN "*.h")

                install(DIRECTORY ${dir} DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/${libname}"
                    FILES_MATCHING PATTERN "*.hpp")
            endif(SPC_ENABLE_INSTALL)

            file(GLOB SOURCES ${dir}/*.cpp)
            file(GLOB HEADERS ${dir}/*.h)
            file(GLOB IMPLS ${dir}/*.hpp)

            list(APPEND LIB_SOURCES ${SOURCES})
            list(APPEND LIB_HEADERS ${HEADERS})
            list(APPEND LIB_IMPLS ${IMPLS})
        endif()
    endforeach()


    file(GLOB SOURCES *.cpp)
    file(GLOB HEADERS *.h)
    file(GLOB IMPLS *.hpp)

    list(APPEND LIB_SOURCES ${SOURCES})
    list(APPEND LIB_HEADERS ${HEADERS})
    list(APPEND LIB_IMPLS ${IMPLS})

    list(APPEND LIB_SOURCES "${ADDITIONAL_SOURCES}")
    message("->>> Additional sources found from within the macro for lib ${libname}:" "${ADDITIONAL_SOURCES}")


    set(sources ${LIB_SOURCES} ${LIB_HEADERS} ${LIB_IMPLS})
    #${HEADERS} ${IMPLS})
    set(libs ${SPC_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${GLOG_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${additional_libs})

    if(NOT SPC_WITH_PCL)
        message("Appending: ${Boost_LIBRARIES}" )
        list(APPEND libs ${Boost_LIBRARIES})
    endif()

    spc_compile_and_link(spc_${libname} "${sources}" "${libs}")
    set_target_properties(spc_${libname} PROPERTIES PUBLIC_HEADER "${HEADERS};${IMPLS}")

    if(SPC_USE_IWYU)
        set_target_properties(spc_${libname} PROPERTIES CXX_INCLUDE_WHAT_YOU_USE "${iwyu_path}")
    endif()

    if(SPC_ENABLE_INSTALL)
        spc_install_target_library_libs(${libname})
    endif()

endmacro()

#add an executable
macro(spc_add_executable name codefiles)
    add_executable(${name} ${codefiles})
    install(TARGETS ${name} EXPORT ${name} RUNTIME DESTINATION "${SPC_INSTALL_BIN_DIR}" COMPONENT bin)
endmacro()

#install a library
macro(spc_install_target_library_libs libname)
    if(SPC_ENABLE_INSTALL)
        install(TARGETS "spc_${libname}"
            EXPORT SPCTargets
            ARCHIVE DESTINATION "${SPC_INSTALL_LIB_DIR}" ## installing static lib in the same place.
            LIBRARY DESTINATION "${SPC_INSTALL_LIB_DIR}"
            COMPONENT shlib
            PUBLIC_HEADER DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/${libname}")
    endif()
endmacro()
