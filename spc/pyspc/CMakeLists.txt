PROJECT(PYSPC)

# macro to add a new pyspc module.
# each module is named following the SPC solib criteria
macro(add_pyspc_module name )
    add_library(${name} SHARED ${name}.cpp ${name}.h)
    set_target_properties(${name} PROPERTIES SUFFIX ${MODULE_SUFFIX} PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${MODULES_OUTPUT_DIR})
    target_link_libraries(${name} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} boost_python  boost_numpy ${SPC_LIBRARIES} io)
endmacro()


# find numpy
find_package(NumPy REQUIRED)


# find python libs, interpreter and boost python (system is needed)
# version is just to have an idea with the versions I am working with
# while deveolping this package
FIND_PACKAGE(PythonInterp 2.7 REQUIRED)
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
FIND_PACKAGE(Boost 1.50.0 REQUIRED  COMPONENTS python system)

#  set includes and solib dirs
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES}) # Deprecated but so convenient!

# some variables
set(MODULES_OUTPUT_DIR "${CMAKE_BINARY_DIR}/pyspc/spc")
set(MODULE_SUFFIX ".so")

# we add the first io module by hand
add_library(io SHARED io.cpp io.h  eigen_numpy.h  eigen_numpy.cpp)
set_target_properties(io PROPERTIES SUFFIX  ${MODULE_SUFFIX} PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${MODULES_OUTPUT_DIR})
target_link_libraries(io ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} boost_python  boost_numpy ${SPC_LIBRARIES} )

# then elements
add_pyspc_module(elements)

# methods
add_pyspc_module(methods)



########## copy init file to build dir #########################################################
set(PYSPC_INIT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py")
add_custom_command( TARGET io
		    POST_BUILD
		    COMMAND ${CMAKE_COMMAND}
			    -E copy_if_different ${PYSPC_INIT_FILE}  ${MODULES_OUTPUT_DIR})
#################################################################################################
