
# add exceptions to rs232 cygwin/mingw/win32 and esd-can


if(DEFINED MSVC)
    set(OS_DEPEND_SRC rs232-vcc.cpp)
elseif(DEFINED WIN32)
    set(OS_DEPEND_SRC rs232-vcc.cpp)
else ()
    set(OS_DEPEND_SRC rs232-cygwin.cpp)
endif () 

IF( WITH_PEAK_CAN )
	set(PEAK_SRC canserial-peak.cpp)
ENDIF()

IF( WITH_ESD_CAN )
	set(ESD_SRC canserial-esd.cpp)
ENDIF()

add_library(
  sdh
  ${OS_DEPEND_SRC}
  ${ESD_SRC}
  ${PEAK_SRC}
  crc.cpp
  dsa.cpp
  sdh.cpp
  sdhbase.cpp
  sdhexception.cpp
  sdhserial.cpp
  serialbase.cpp
  simplestringlist.cpp
  simplevector.cpp
  unit_converter.cpp
  util.cpp
)
  		


