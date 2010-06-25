# Locate Lua library
# This module defines
#  TOLUA_FOUND, if false, do not try to link to Lua 
#  TOLUA_LIBRARIES
#  TOLUA_INCLUDE_DIR, where to find tolua++.h
#  TOLUA_CMD,   
#
# defines macro ADD_TOLUA_PACKAGE(packagename stubname), 
# appends to SRC_FILES and SRC_FILES_HPP

MACRO (ADD_TOLUA_PACKAGE pkgname stubname)

ADD_CUSTOM_COMMAND(
  # Because we are building in a temporary directory, we have to use absolute
  # paths here.
  OUTPUT
  "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.hpp"

  COMMAND ${TOLUA_CMD}
  -o "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp"
  -H "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.hpp"
  -n "${pkgname}" "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}.pkg"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}.pkg")

LIST(APPEND SRC_FILES
  "${pkgname}.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp"
)

LIST(APPEND SRC_FILES_HPP  
  "${pkgname}.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp"
)
ENDMACRO (ADD_TOLUA_PACKAGE)


FIND_PATH(TOLUA_INCLUDE_DIR tolua++.h
  HINTS
  ${TOLUA_DIR}
  $ENV{TOLUA_DIR}
  PATH_SUFFIXES include/tolua include
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt
)

FIND_LIBRARY(TOLUA_LIBRARY 
  NAMES tolua++ tolua++5.1 tolua++51 tolua tolua51 tolua5.1
  HINTS
  ${TOLUA_DIR}  
  $ENV{TOLUA_DIR}
  ${TOLUA_LIBRARY_DIR}
  PATH_SUFFIXES lib64 lib
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw
  /opt/local
  /opt/csw
  /opt
)

SET( TOLUA_LIBRARIES ${TOLUA_LIBRARY})

INCLUDE(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LUA_FOUND to TRUE if 
# all listed variables are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Tolua++ DEFAULT_MSG TOLUA_LIBRARIES TOLUA_INCLUDE_DIR)

MARK_AS_ADVANCED(TOLUA_INCLUDE_DIR TOLUA_LIBRARIES TOLUA_LIBRARY)