
# check compiler/operating system

set(LibraryList
  rwhw_sdh
  #rwhw_can 
  #rwhw_serialport
  sdh
  ntcan.lib # needed by rwhw_can
  ${ROBWORK_LIBRARIES}
)

add_executable(SDHExample1 SDHExample1.cpp sdhoptions.cpp)
target_link_libraries(SDHExample1 ${LibraryList})

add_executable(demo-simple2 demo-simple2.cpp sdhoptions.cpp)
target_link_libraries(demo-simple2 sdh ntcan.lib)