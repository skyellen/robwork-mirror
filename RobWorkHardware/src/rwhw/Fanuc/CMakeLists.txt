
if (DEFINED WIN32)
  set(FanucFile ./FanucDriver.cpp)
  add_library(
      rwhw_fanucdevice
      ${FanucFile}
      FanucVirtual.cpp
      VelRampProfile.cpp
  )
  set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_fanucdevice;")
else ()
    message("Warning: Fanuc device will not be build.")
  set(FanucFile )
endif ()

