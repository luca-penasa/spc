# include CMakePackageConfigHelpers macro
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
# set version
set(version 0.0.1)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/SPCConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
)


set(SPC_HAS_PCL ${SPC_WITH_PCL})
set(SPC_MODULES ${SPC_LIBRARIES})


# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/SPCConfig.in.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/SPCConfig.cmake"
  INSTALL_DESTINATION lib/cmake/spc
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# install config files
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/cmake/SPCConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/cmake/SPCConfigVersion.cmake"
        DESTINATION lib/cmake/spc
)


