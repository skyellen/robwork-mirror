
# add exceptions to rs232 cygwin/mingw/win32 and esd-can

if (DEFINED CYGWIN)
    set(OS_DEPEND_SRC rs232-cygwin.cpp)
elseif(DEFINED MSVC)
    set(OS_DEPEND_SRC rs232-vcc.cpp)
elseif(DEFINED WIN32)
    set(OS_DEPEND_SRC rs232-vcc.cpp)
else()
    set(OS_DEPEND_SRC rs232-cygwin.cpp)
endif () 

add_library(
  sdh
  ${OS_DEPEND_SRC}
  canserial-esd.cpp
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
  		


