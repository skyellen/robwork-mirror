# check compiler/operating system

IF(COMPONENT_pcube_ENABLE)
    set(LibraryList
      rwhw_pcube 
      rwhw_can 
      ntcan.lib
      rwhw_serialport
      ${ROBWORK_LIBRARIES}
    )
    
    LINK_DIRECTORIES(${ESDCAN_LIB_DIR})
    
    add_executable(PCubeExample PCubeExample.cpp )
    target_link_libraries(PCubeExample ${LibraryList})
ENDIF(COMPONENT_pcube_ENABLE)
