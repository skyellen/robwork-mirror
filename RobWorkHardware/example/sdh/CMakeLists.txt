# check compiler/operating system



IF(COMPONENT_sdh_ENABLE AND SDH_FOUND AND PEAKCAN_FOUND)
    FIND_PACKAGE(SDH)
    FIND_PACKAGE(PEAKCAN)
    INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} )

    set(LibraryList
      rwhw_sdh
      ${SDH_LIBRARY}
      ${PEAKCAN_LIBRARIES}
      ${ROBWORK_LIBRARIES}
    )
    
    add_executable(SDHExample1 SDHExample1.cpp sdhoptions.cpp)
    target_link_libraries(SDHExample1 ${LibraryList})
    
    add_executable(demo-simple2 demo-simple2.cpp sdhoptions.cpp)
    target_link_libraries(demo-simple2 ${LibraryList})
ENDIF(COMPONENT_sdh_ENABLE AND SDH_FOUND AND PEAKCAN_FOUND)
