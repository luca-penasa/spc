project(qGEOVis)


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


#    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    add_library( qGEOVis SHARED ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list})

  target_link_libraries(${PROJECT_NAME} PclUtils ${SPC_LIBRARIES} ${QT_LIBRARIES})

  install(TARGETS ${PROJECT_NAME} DESTINATION lib)
