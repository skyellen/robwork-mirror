
if (DEFINED WIN32)
  set(FanucFile ./FanucDriver.cpp)
else ()
    message("Warning: Fanuc device will not be build.")
  set(FanucFile )
endif ()

add_library(
  rwhw_fanucdevice
  ${FanucFile}
  FanucVirtual.cpp
  VelRampProfile.cpp
)
