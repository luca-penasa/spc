cmake_minimum_required(VERSION 2.8)
project(qGEO)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11 -fopenmp -fPIC" )
    include(${CC_SOURCE_DIR}/CMakeInclude.cmake)
    include(${CC_SOURCE_DIR}/CMakeExternalLibs.cmake)




    set(CC_PLUGINS_SOURCE_DIR "${CC_SOURCE_DIR}/qCC/plugins")
    set(CC_PLUGINS_BUILD_DIR "${CC_BUILD_DIR}/qCC/plugins")

   link_directories(${CC_BUILD_DIR}/lib)


    #for mocketized files
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/geoutils)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/geoutils/dialogs)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    include_directories(${SPC_BUILD_INCLUDE_DIRS})


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


    file( GLOB header_list *.h)
    file( GLOB source_list *.cpp)
    file( GLOB ui_list *.ui )
    file( GLOB qrc_list *.qrc )
    file( GLOB rc_list *.rc )

    # find Qt mocable files
    find_mocable_files( mocable_list ${header_list} )
    qt4_wrap_cpp( moc_list ${mocable_list} )
    QT4_WRAP_UI( generated_ui_list ${ui_list} )
    QT4_ADD_RESOURCES( generated_qrc_list ${qrc_list} )

    add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list})


  #pcl
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})



  #include( ${CC_PLUGINS_SOURCE_DIR}/CMakePluginTpl.cmake )
  target_link_libraries(${PROJECT_NAME} PclUtils geoutils ${QWT_LIBRARIES} ${SPC_LIBRARIES} ${QT_LIBRARIES})

  install(TARGETS ${PROJECT_NAME} DESTINATION ${QGEO_INSTALL_DIR})
