
add_library(
  rwhw_serialport
  SerialPort.cpp
)

set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_serialport;")