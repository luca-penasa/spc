#compiles and link a spc_lib
macro(spc_compile_and_link name files libs)
    if(BUILD_SPC_SHARED)
            add_library(${name} SHARED ${files})
    else()
            add_library(${name} STATIC ${files})
    endif()

    target_link_libraries(${name} ${libs})

#    spc_install_target_library_libs(${name})

    #update the variable keeping the libarires
    set(SPC_LIBRARIES ${SPC_LIBRARIES} ${name} CACHE INTERNAL "SPC libs")
    set(SPC_BUILD_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "SPC include dirs at build time")
    message("${SPC_LIBRARIES}")
endmacro()


#macro(spc_set_properties_public_header headers)
#    set_target_properties(${name} PROPERTIES PUBLIC_HEADER "${headers}")
#endmacro()

# this macro add a library to SPC
macro(spc_add_library)
    get_filename_component(libname ${CMAKE_CURRENT_SOURCE_DIR}  NAME)

    file(GLOB_RECURSE SOURCES *.cpp)
    file(GLOB_RECURSE HEADERS *.h)
    file(GLOB_RECURSE IMPLS *.hpp)

    set(sources ${SOURCES} ${HEADERS} ${IMPLS})
    set(libs ${SPC_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${additional_libs})
    spc_compile_and_link(spc_${libname} "${sources}" "${libs}")
    set_target_properties(spc_${libname} PROPERTIES PUBLIC_HEADER "${HEADERS};${IMPLS}")

    spc_install_target_library_libs(${libname})
endmacro()

##http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
#MACRO(SUBDIRLIST result curdir)
#  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
#  SET(dirlist "")
#  FOREACH(child ${children})
#    IF(IS_DIRECTORY ${curdir}/${child})
#        SET(dirlist ${dirlist} ${child})
#    ENDIF()
#  ENDFOREACH()
#  SET(${result} ${dirlist})
#ENDMACRO()


#add an executable
macro(spc_add_executable name codefiles)
    add_executable(${name} ${codefiles})
    install(TARGETS ${name} EXPORT ${name} RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endmacro()

##install hedears
#macro(spc_install_target_library_headers libname headers)
#    install(FILES ${headers} DESTINATION include/spc/${libname})
#endmacro()

#install a library
macro(spc_install_target_library_libs libname)
    install(TARGETS "spc_${libname}"
            EXPORT SPCTargets
            ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" ## installing static lib in the same place.
            LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
            COMPONENT shlib
            PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/spc/${libname}")
endmacro()


# find all moc-able files in current folder
function( spc_find_mocable_files in_headers out_headers)
    set(local_list)
    foreach( one_file ${in_headers} )
        file( READ ${one_file} stream )
        if( stream MATCHES "Q_OBJECT" )
            list( APPEND local_list ${one_file})
        endif()
    endforeach()
    set(${out_headers} ${local_list} PARENT_SCOPE)
endfunction()
