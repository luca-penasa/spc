macro(collect_sources)
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

    set(sources ${LIB_SOURCES} ${LIB_HEADERS} ${LIB_IMPLS})
endmacro()

# this macro add a library to SPC
macro(spc_add_library libname)
    get_filename_component(libname ${CMAKE_CURRENT_SOURCE_DIR}  NAME)
    set(target_name "${libname}")

    collect_sources()


    if(SPC_BUILD_SHARED)
        add_library(${target_name} SHARED ${sources})
    else()
        add_library(${target_name} STATIC ${sources})
    endif()
    add_library(spc::${libname} ALIAS ${target_name})


    target_link_libraries(${target_name} PUBLIC ${SPC_LINK_TARGETS} ${libs})
    target_compile_definitions(${target_name} PUBLIC EIGEN_MATRIXBASE_PLUGIN=<spc/core/eigen_matrix_base_plugin.h>)
    #    target_compile_definitions(${name} PUBLIC ${PCL_DEFINITIONS})
    target_compile_definitions(${target_name} PUBLIC ${SPC_COMPILER_DEFINITIONS})

    message(" COMPILER OPTIONS ${SPC_COMPILER_OPTIONS}")
#    set_target_properties(${target_name} PROPERTIES PREFIX "libspc")
target_compile_options(${target_name} PUBLIC ${SPC_COMPILER_OPTIONS})
target_compile_features(${target_name} PUBLIC ${SPC_COMPILER_FEATURES})

    # TODO split build and install interface
    target_sources(${target_name} PUBLIC "${CMAKE_SOURCE_DIR}/submodules/easyloggingpp/src/easylogging++.cc")

    set(SPC_MODULES ${SPC_MODULES} ${target_name} CACHE INTERNAL "SPC libs")
    #    set(SPC_BUILD_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "SPC include dirs at build time")


    target_include_directories(${target_name}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty> # TODO add
        "$<BUILD_INTERFACE:${SPC_SUBMODULES_INCLUDES}>"
#        ${SPC_SUBMODULES_INCLUDES}
#        PRIVATE
#        ${CMAKE_SOURCE_DIR}
#        ${SPC_SUBMODULES_INCLUDES}


        )

    message("ABCC ${SPC_SUBMODULES_INCLUDES}")
    #    set_target_properties(${target_name} PROPERTIES PUBLIC_HEADER "${HEADERS};${IMPLS}")


    if(SPC_USE_IWYU)
        set_target_properties(${target_name} PROPERTIES CXX_INCLUDE_WHAT_YOU_USE "${iwyu_path}")
    endif()

    export(TARGETS ${target_name} NAMESPACE spc:: FILE ${PROJECT_BINARY_DIR}/cmake/${libname}-export.cmake)


    if(SPC_ENABLE_INSTALL)
        spc_install_target_library_libs(${target_name})
    endif()

endmacro()

#add an executable
macro(spc_add_executable name codefiles)
    add_executable(${name} ${codefiles} )
    target_link_libraries(${name} ${SPC_LINK_TARGETS} ${SPC_MODULES})
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
