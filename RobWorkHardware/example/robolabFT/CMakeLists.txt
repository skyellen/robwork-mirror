# check compiler/operating system

IF(COMPONENT_robolabFT_ENABLE )
    FIND_PACKAGE(robolabFT)

    INCLUDE_DIRECTORIES( ${ROBOLABFT_INCLUDE_DIR} )

    set(LibraryList
      rwhw_robolabFT
      ${ROBWORK_LIBRARIES}
    )
    
    add_executable(robolabFT robolabFT.cpp)
    target_link_libraries(robolabFT ${LibraryList} rwhw_serialport -lpthread )
    
ENDIF(COMPONENT_robolabFT_ENABLE)
