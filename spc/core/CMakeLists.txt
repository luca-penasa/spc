############ COre lib needs boost regex library;
#set(Boost_USE_STATIC_LIBS ON)

find_package(Boost REQUIRED COMPONENTS  regex)


if (Boost_FOUND)
#    set(SPC_LIBRARIES "${SPC_LIBRARIES} ${Boost_LIBRARIES}")
    message("BOOST REGEX ${Boost_LIBRARIES}" )
#    set(SPC_LIBRARIES ${SPC_LIBRARIES} ${Boost_LIBRARIES} CACHE INTERNAL "SPC libs")
endif()


set(additional_libs ${Boost_LIBRARIES})


spc_add_library()








