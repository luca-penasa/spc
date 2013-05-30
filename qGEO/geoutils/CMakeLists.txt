cmake_minimum_required (VERSION 2.8)
project(geoutils)

#lidarlib
include_directories(/home/luca/apps/include)
link_directories(/home/luca/apps/lib)

include_directories( ${CC_DLL_SOURCE_DIR}/include )
include_directories( ${CloudCompare_SOURCE_DIR} )
include_directories( ${CloudCompare_SOURCE_DIR}/db_tree )
include_directories( ${CloudCompare_BINARY_DIR} )
include_directories( ${QCC_DB_DLL_SOURCE_DIR} )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/dialogs)







find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
add_definitions(${VTK_DEFINITIONS})


include_directories( ${Qwt6_INCLUDE_DIR} )

file( GLOB header_list *.h dialogs/*.h)
file( GLOB source_list *.cpp dialogs/*.cpp)
file( GLOB ui_list *.ui dialogs/*.ui)
file( GLOB qrc_list *.qrc )
file( GLOB rc_list *.rc )

# find Qt mocable files
find_mocable_files( mocable_list ${header_list} )
qt4_wrap_cpp( moc_list ${mocable_list} )
QT4_WRAP_UI( generated_ui_list ${ui_list} )
QT4_ADD_RESOURCES( generated_qrc_list ${qrc_list} )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${QWT_LIBRARY} ${Qwt6_Qt4_LIBRARY} ${VTK_LIBRARIES} ${SPC_LIBRARIES} PclUtils)
