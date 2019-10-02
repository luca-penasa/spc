find_package(Doxygen)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/doc/spc)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_SOURCE_DIR}/spc/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ALL
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc/spc
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)
