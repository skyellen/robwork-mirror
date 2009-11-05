
if (NOT DEFINED WIN32)
add_library(
  rwhw_swissranger
  SRCalibrationData.cpp
  SwissRanger.cpp
)
endif()
