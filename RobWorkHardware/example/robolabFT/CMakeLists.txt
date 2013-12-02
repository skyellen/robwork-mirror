# check compiler/operating system

FIND_PACKAGE(robolabFT)


IF(COMPONENT_robolabFT_ENABLE )
    INCLUDE_DIRECTORIES( ${ROBOLABFT_INCLUDE_DIR} )

    set(LibraryList
      rwhw_robolabFT
      ${ROBWORK_LIBRARIES}
    )
    
    add_executable(robolabFT robolabFT.cpp)
    target_link_libraries(robolabFT ${LibraryList} rwhw_serialport -lpthread )
    
ENDIF(COMPONENT_robolabFT_ENABLE)
