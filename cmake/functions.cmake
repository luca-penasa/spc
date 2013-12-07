#compiles and link something
macro(spc_compile_and_link name files libs)
    if(BUILD_SPC_SHARED)
            add_library(${name} SHARED ${files})
    else()
            add_library(${name} STATIC ${files})
    endif()
    target_link_libraries(${name} ${libs})
    spc_install_target_library_libs (${name})

    #update the variable keeping the libarires

    set(SPC_LIBRARIES ${SPC_LIBRARIES} ${name} CACHE INTERNAL "SPC libs")
    set(SPC_BUILD_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "SPC include dirs at build time")
    message("${SPC_LIBRARIES}")
endmacro()

# this macro add a library to SPC
macro(spc_add_library libname dependence)

    file(GLOB SOURCES *.cpp)
    file(GLOB HEADERS *.h)
    file(GLOB IMPLS *.hpp)
    file(GLOB UIS *.ui) #this when compiling
    file(GLOB QRCS *.qrc) #qt resources ?

    #find also if there are any mocable file
    spc_find_mocable_files("${HEADERS}" MOCABLES)


    if(UIS OR QRCS  OR MOCABLES)
        #we obviously need to link and use qt
        spc_library_needs_qt()
        QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})
        QT4_WRAP_UI(HEADERS_UIS ${UIS})
        QT4_ADD_RESOURCES(QRCS_RES ${QRCS})
        include_directories(${CMAKE_CURRENT_BINARY_DIR})

    endif()

    set(sources ${SOURCES} ${HEADERS} ${IMPLS} ${HEADERS_MOC} ${HEADERS_UIS} ${QRCS_RES})
    set(libs ${dependence} ${SCP_LIBRARIES})
    spc_compile_and_link(spc_${libname} "${sources}" "${libs}")

    set(all_heads ${HEADERS} ${IMPLS})
    spc_install_target_library_headers(${libname} "${all_heads}")

endmacro()

#can be called when a module needs PCL
macro(spc_library_needs_pcl)
    find_package(PCL 1.6 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    set(additional_libs ${additional_libs} ${PCL_LIBRARIES})
endmacro()

#to be called if a module need QT
macro(spc_library_needs_qt)
    find_package(Qt4 REQUIRED)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})
    set(additional_libs ${additional_libs} ${QT_LIBRARIES})
endmacro()

#add an executable
macro(spc_add_executable name codefiles)
    add_executable(${name} ${codefiles})
    install(TARGETS ${name} DESTINATION bin)
endmacro()

#install hedears
macro(spc_install_target_library_headers libname headers)
    install(FILES ${headers} DESTINATION include/spc/${libname})
endmacro()

#install a library
macro(spc_install_target_library_libs libname)
    install(TARGETS "${libname}" DESTINATION lib)
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
