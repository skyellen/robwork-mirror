if (NOT DEFINED WIN32)
add_library(
  rwhw_motomanIA20
  MotomanIA20.cpp
)
set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_motomanIA20;")
endif()