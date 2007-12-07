
if (DEFINED WIN32)
  set(CanFiles ./canbus/IEICAN02/icanapi.cpp
               ./canbus/IEICAN02/IEICANPort.cpp
               ./canbus/ESDCAN/ESDCANPort.cpp)
else ()
  set(CanFiles )
endif ()

add_library(
  rw_io
  ${RW_IO_USER_FILES}
  ${CanFiles}
  ./serialport/SerialPort.cpp)

