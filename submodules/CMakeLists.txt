
if(CEREAL_IS_INTERNAL)
    if(SPC_ENABLE_INSTALL)
        INSTALL( DIRECTORY cereal/include/cereal DESTINATION "${INSTALL_INCLUDE_DIR}/spc/3rdParty" )
    endif()
endif()

