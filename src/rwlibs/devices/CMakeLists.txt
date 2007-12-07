set(Ext ${PROJECT_SOURCE_DIR}/ext)

include_directories(${Ext}/katana/include)

if (DEFINED WIN32)
  set(FanucFile ./Fanuc/FanucDriver.cpp)
else ()
    message("Warning: Fanuc device will not be build.")
  set(FanucFile )
endif ()

# The dockwelder device currently can't be built with cygwin because of some
# probably trivial issues related to boost::asio.
if (DEFINED CYGWIN)
    message("Warning: Dockwelder device will not be build.")
  set(DocwelderFile )
else ()
  set(DocwelderFile ./dockwelder/DockWelder.cpp)
endif ()

# We currently can't compile these directories with neither Cygwin nor
# Microsoft compilers:
if (NOT DEFINED CYGWIN AND NOT DEFINED MSVC)
  set(orodependantfiles 
    ./pa10/PA10.cpp
    ./pa10/PA10Driver.cpp
    ./pa10/PA10Virtual.cpp)
else ()
  message("Warning: pa10 and panascara devices will not be build.")
endif()

add_library(
  rw_devices
  ${RW_DEVICES_USER_FILES}
  ${orodependantfiles}
  ${DocwelderFile}
  ${FanucFile}
  ./crsA465/CRSA465.cpp
  ./crsA465/crs_aci.cpp
  ./Fanuc/FanucVirtual.cpp
  ./Fanuc/VelRampProfile.cpp
  ./katana/Katana.cpp
  ./PowerCube/Cube.cpp
  ./PowerCube/PCubeProtocol.cpp)
