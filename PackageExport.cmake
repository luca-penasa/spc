################################## INSTALL OPTIONS #####################################################
set(SPC_INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(SPC_INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(SPC_INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

##### def dir for cmake files on install
if(WIN32 AND NOT CYGWIN)
    set(SPC_DEF_INSTALL_CMAKE_DIR CMake)
else()
    set(SPC_DEF_INSTALL_CMAKE_DIR lib/CMake/SPC)
endif()

set(INSTALL_CMAKE_DIR ${SPC_DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
    set(var SPC_INSTALL_${p}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

# The interesting stuff goes here
# ===============================


#list(APPEND SPC_LIBRARIES ${AQUILA_LIBRARIES})
# Add all targets to the build-tree export set - also binaries target should be added
export(TARGETS ${SPC_LIBRARIES} FILE "${PROJECT_BINARY_DIR}/SPCTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE SPC)

#### BUILD TREE
## ... for the build tree - to be done!
set (CONF_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
set (CONF_CEREAL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/submodules/cereal/include")
set (CONF_LIBRARY_DIRS "${LIBRARY_OUTPUT_PATH}")
set (CONF_NANOFLANN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/submodules/nanoflann/include")
set (CONF_EASYLOGGINGPP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/submodules/easyloggingpp/src")
set(SPC_ADDITIONAL_CMAKE_PATHS "${PROJECT_BINARY_DIR}/cmake")


configure_file(SPCConfig.cmake.in "${PROJECT_BINARY_DIR}/SPCConfig.cmake" @ONLY)

## the ConfigVersion.cmake
configure_file(SPCConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/SPCConfigVersion.cmake" @ONLY)

file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/cmake")
file(COPY  "${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake"
	 DESTINATION "${PROJECT_BINARY_DIR}/cmake")


#### INSTALL TREE
if(SPC_ENABLE_INSTALL)
    ## reset conf_include_dirs for the install tree
    set (CONF_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}")
    set (CONF_CEREAL_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty")
    set (CONF_NANOFLANN_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty")
    set (CONF_LIBRARY_DIRS "${SPC_INSTALL_LIB_DIR}")
    set (CONF_EASYLOGGINGPP_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty/easiloggingpp/src")

	set(SPC_ADDITIONAL_CMAKE_PATHS "${INSTALL_CMAKE_DIR}")


    ## reconfigure the SPCConfig.cmake putting it in a subdirectory of the build tree
    configure_file(SPCConfig.cmake.in "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/SPCConfig.cmake" @ONLY)


    # Install the FooBarConfig.cmake and FooBarConfigVersion.cmake in the install tree
    install(FILES "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/SPCConfig.cmake"
        "${PROJECT_BINARY_DIR}/SPCConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

    ## Install the export set for use with the install-tree
    install(EXPORT SPCTargets DESTINATION
        "${INSTALL_CMAKE_DIR}" COMPONENT dev)

	install(FILES "${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake"
		DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

    INSTALL( DIRECTORY submodules/easyloggingpp DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty" )
endif()
