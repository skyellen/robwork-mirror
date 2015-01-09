# - Try to find CMU1394 include dirs and libraries
# Usage of this module as follows:
#
#     FIND_PACKAGE( CMU1394 )
#
#######################################################################################
#
# Variables that can be used by this module
#
#  CMU1394_ROOT                 Preferred installation prefix for searching for CMU1394,
#                              set this if the module has problems finding the proper CMU1394 installation
#  CMU1394_INCLUDEDIR           Set this to the include directory of CMU1394, if the
#                              module has problems finding the proper CMU1394 installation
#  CMU1394_LIBRARYDIR           Set this to the lib directory of CMU1394, if the
#                              module has problems finding the proper CMU1394 installation
# 
#  The last three variables are available also as environment variables
#
# 
# Variables defined by this module:
#
#  CMU1394_FOUND                         System has CMU1394, this means the include dir was found,
#                                       as well as all the libraries specified in the COMPONENTS list
#  CMU1394_INCLUDE_DIRS                  CMU1394 include directories, not cached
#  CMU1394_INCLUDE_DIR                   This is almost the same as above, but this one is cached and may be
#                                       modified by advanced users
#  CMU1394_LIBRARIES                     Link these to use the CMU1394 libraries that you specified, not cached
#  CMU1394_LIBRARY_DIRS                  The path to where the CMU1394 library files are.


SET( _CMU1394_IN_CACHE TRUE)
IF(CMU1394_INCLUDE_DIR)
 #FIND_LIBRARY(CMU1394 1394camera.lib ) #  1394camera.dll 1394camera.dll 1394camera.dll
ELSE(CMU1394_INCLUDE_DIR)
  SET( _CMU1394_IN_CACHE FALSE)
ENDIF(CMU1394_INCLUDE_DIR)

IF (_CMU1394_IN_CACHE)
  # in cache already
  SET(CMU1394_FOUND TRUE)
  SET(CMU1394_INCLUDE_DIRS ${CMU1394_INCLUDE_DIR})
ELSE (_CMU1394_IN_CACHE)

  SET(_CMU1394_INCLUDE_SEARCH_DIRS
    C:/CMU/1394Camera/include/
    "C:/CMU/1394Camera/"
    "$ENV{ProgramFiles}/CMU/1394Camera/include/"
    "$ENV{ProgramFiles}/CMU/1394Camera/"
  )

  SET(_CMU1394_LIBRARIES_SEARCH_DIRS
    C:/CMU/1394Camera/lib/
    "C:/CMU/1394Camera/"
    "$ENV{ProgramFiles}/CMU/1394Camera/lib/"
    "$ENV{ProgramFiles}/CMU/1394Camera/"
  )
   
     # If CMU1394_ROOT was defined in the environment, use it.
  if (NOT CMU1394_ROOT AND NOT $ENV{CMU1394_ROOT} STREQUAL "")
    set(CMU1394_ROOT $ENV{CMU1394_ROOT})
  endif(NOT CMU1394_ROOT AND NOT $ENV{CMU1394_ROOT} STREQUAL "")

  # If CMU1394ROOT was defined in the environment, use it.
  if (NOT CMU1394_ROOT AND NOT $ENV{CMU1394ROOT} STREQUAL "")
    set(CMU1394_ROOT $ENV{CMU1394ROOT})
  endif(NOT CMU1394_ROOT AND NOT $ENV{CMU1394ROOT} STREQUAL "")

  # If CMU1394_INCLUDEDIR was defined in the environment, use it.
  IF( NOT $ENV{CMU1394_INCLUDEDIR} STREQUAL "" )
    set(CMU1394_INCLUDEDIR $ENV{CMU1394_INCLUDEDIR})
  ENDIF( NOT $ENV{CMU1394_INCLUDEDIR} STREQUAL "" )

  # If CMU1394_LIBRARYDIR was defined in the environment, use it.
  IF( NOT $ENV{CMU1394_LIBRARYDIR} STREQUAL "" )
    set(CMU1394_LIBRARYDIR $ENV{CMU1394_LIBRARYDIR})
  ENDIF( NOT $ENV{CMU1394_LIBRARYDIR} STREQUAL "" )
   
  IF( CMU1394_ROOT )
    file(TO_CMAKE_PATH ${CMU1394_ROOT} CMU1394_ROOT)
    SET(_CMU1394_INCLUDE_SEARCH_DIRS 
      ${CMU1394_ROOT}/include 
      ${CMU1394_ROOT}
      ${_CMU1394_INCLUDE_SEARCH_DIRS})
    SET(_CMU1394_LIBRARIES_SEARCH_DIRS 
      ${CMU1394_ROOT}/lib 
      ${CMU1394_ROOT} 
      ${_CMU1394_LIBRARIES_SEARCH_DIRS})
  ENDIF( CMU1394_ROOT )

  IF( CMU1394_INCLUDEDIR )
    file(TO_CMAKE_PATH ${CMU1394_INCLUDEDIR} CMU1394_INCLUDEDIR)
    SET(_CMU1394_INCLUDE_SEARCH_DIRS 
      ${CMU1394_INCLUDEDIR} ${_CMU1394_INCLUDE_SEARCH_DIRS})
  ENDIF( CMU1394_INCLUDEDIR )

  IF( CMU1394_LIBRARYDIR )
    file(TO_CMAKE_PATH ${CMU1394_LIBRARYDIR} CMU1394_LIBRARYDIR)
    SET(_CMU1394_LIBRARIES_SEARCH_DIRS 
      ${CMU1394_LIBRARYDIR} ${_CMU1394_LIBRARIES_SEARCH_DIRS})
  ENDIF( CMU1394_LIBRARYDIR )

  IF( NOT CMU1394_INCLUDE_DIR )
    FIND_PATH(CMU1394_INCLUDE_DIR
      NAMES         1394camapi.h
      HINTS         ${_CMU1394_INCLUDE_SEARCH_DIRS}
      PATH_SUFFIXES ${_CMU1394_PATH_SUFFIXES}
    )
  ENDIF( NOT CMU1394_INCLUDE_DIR )
  
  IF(CMU1394_INCLUDE_DIR)
      #MESSAGE("INCLUDE DIR FOU ND!")
  ELSE(CMU1394_INCLUDE_DIR)
      #MESSAGE("INCLUDE DIR NOT FOUND")
    set(CMU1394_ERROR_REASON
      "${CMU1394_ERROR_REASON}Unable to find the Boost header files. Please set BOOST_ROOT to the root directory containing Boost or BOOST_INCLUDEDIR to the directory containing Boost's headers.")
  ENDIF(CMU1394_INCLUDE_DIR)
  
  FIND_LIBRARY(CMU1394_LIBRARY_RELEASE
          NAMES 1394camera HINTS  
          ${_CMU1394_LIBRARIES_SEARCH_DIRS}
  )

  FIND_LIBRARY(CMU1394_LIBRARY_DEBUG
          NAMES 1394camerad HINTS  
          ${_CMU1394_LIBRARIES_SEARCH_DIRS}
  )
  
  SET(CMU1394_INCLUDE_DIRS ${CMU1394_INCLUDE_DIR})

  SET(CMU1394_FOUND FALSE)
  IF(CMU1394_INCLUDE_DIR)
    SET( CMU1394_FOUND TRUE )
  ELSE(CMU1394_INCLUDE_DIR)
    SET( CMU1394_FOUND FALSE)
  ENDIF(CMU1394_INCLUDE_DIR)
  
  IF (CMU1394_FOUND)
    IF (NOT CMU1394_FIND_QUIETLY)
      #MESSAGE(STATUS "CMU1394: found!")
    ENDIF(NOT CMU1394_FIND_QUIETLY)
    IF(CMAKE_BUILD_TYPE EQUAL "DEBUG")
        SET(CMU1394_LIBRARIES ${CMU1394_LIBRARIES} CMU1394_LIBRARY_DEBUG)
    ELSE()
        SET(CMU1394_LIBRARIES ${CMU1394_LIBRARIES} CMU1394_LIBRARY_RELEASE)    
    ENDIF()
    
  ELSE (CMU1394_FOUND)
    IF (CMU1394_FIND_REQUIRED)
      MESSAGE(SEND_ERROR "Unable to find the requested CMU1394 libraries.\n${CMU1394_ERROR_REASON}")
    ENDIF(CMU1394_FIND_REQUIRED)
  ENDIF(CMU1394_FOUND)
  
   # show the CMU1394_INCLUDE_DIRS AND CMU1394_LIBRARIES variables only in the advanced view
  MARK_AS_ADVANCED(CMU1394_INCLUDE_DIR CMU1394_INCLUDE_DIRS CMU1394_LIBRARY_DIRS)
  
ENDIF(_CMU1394_IN_CACHE)
