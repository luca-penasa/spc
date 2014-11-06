
if(CEREAL_IS_INTERNAL)
    if(SPC_ENABLE_INSTALL)
        INSTALL( DIRECTORY cereal/include/cereal DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty" )
        INSTALL( FILES nanoflann/include/nanoflann.hpp DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty" )

    endif()
endif()

