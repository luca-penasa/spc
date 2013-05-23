cmake_minimum_required(VERSION 2.8)
project(qGEO)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11 -fopenmp -fPIC" )
    include(${CC_SOURCE_DIR}/CMakeInclude.cmake)
    include(${CC_SOURCE_DIR}/CMakeExternalLibs.cmake)

#we need qwt ############
  find_path ( QWT_INCLUDE_DIR qwt_plot.h /usr/include/qwt6 )
  find_library ( QWT_LIBRARIES NAMES qwt6) #explicit look for qwt6

  include ( FindPackageHandleStandardArgs )
  find_package_handle_standard_args( QWT DEFAULT_MSG QWT_LIBRARIES QWT_INCLUDE_DIR )

  if (${QWT_FOUND})
    mark_as_advanced(QWT_LIBRARIES QWT_INCLUDE_DIR )
    add_definitions( -DCC_QWT_SUPPORT )
    include_directories(${QWT_INCLUDE_DIR})
  endif()
#################


    set(CC_PLUGINS_SOURCE_DIR "${CC_SOURCE_DIR}/qCC/plugins")
    set(CC_PLUGINS_BUILD_DIR "${CC_BUILD_DIR}/qCC/plugins")

   link_directories(${CC_BUILD_DIR}/lib)


    #for mocketized files
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/geoutils)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/geoutils/dialogs)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})


    include_directories(${CC_SOURCE_DIR}/libs/qCC_db/)
    include_directories(${CC_SOURCE_DIR}/qCC/db_tree/)

    include_directories(${CC_SOURCE_DIR}/qCC/)
    include_directories(${CC_SOURCE_DIR}/qCC/dialogs)

    include_directories(${CC_PLUGINS_SOURCE_DIR})



    include_directories(${CC_BUILD_DIR}/qCC) #for mocketized files that cc created

    include_directories(${CC_PLUGINS_SOURCE_DIR}) # in this way we can get acces to headers from other plugins
    include_directories(${CC_SOURCE_DIR}/CC/include/)
    #include_directories(${QGEO_PLUGIN_DLL_BINARY_DIR}/geoutils) #for including moc'ed headers
    include_directories(geoutils)
    #include_directories(${QPCL_PLUGIN_DLL_SOURCE_DIR}/../) #for accessing other plugins headers

    add_subdirectory(geoutils)
    add_subdirectory(tools)

    list(APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CC_PLUGINS_SOURCE_DIR}/qPCL/PclUtils/utils/cc2sm.cpp
                                             ${CC_PLUGINS_SOURCE_DIR}/qPCL/PclUtils/utils/sm2cc.cpp)



  #pcl
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})



  include( ${CC_PLUGINS_SOURCE_DIR}/CMakePluginTpl.cmake )


    message(${QGEO_INSTALL_DIR})

  target_link_libraries(${PROJECT_NAME} PclUtils geoutils ${Qwt6_Qt4_LIBRARY} )
  install(TARGETS ${PROJECT_NAME} DESTINATION /home/luca/bin/plugins)
