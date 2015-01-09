# Try to find CMU 1394 (firewire) digital camera driver library  
#
# Available from:
# www-2.cs.cmu.edu/~iwan/1394/
#
# Once run this will define: 
#
# 1394CAMERACMU_FOUND
# CMU1394_INCLUDE_DIR
# CMU1394_LIBRARIES
# 1394CAMERACMU_HEADER       = This is a hack to allow command completion in MSVS
#
#  (1394CAMERACMU_LINK_DIRECTORIES: not yet...)
# 
# NOTES
# tested with version 6.2, 6.3 (on Windows XP)
#
# AUTHOR
# Jan Woetzel <http://www.mip.informatik.uni-kiel.de/~jw> (02/2004)


MESSAGE(STATUS "Find1394CameraCMU.cmake: Please note that 1394CameraCMU vars. were renamed to uppercase 1394CAMERACMU --")


IF (NOT WIN32)
  #MESSAGE(STATUS "Find1394CameraCMU.cmake: This library supports only WIN32. skipping.")
  SET(1394CAMERACMU_FOUND FALSE)
    
  
ELSE (NOT WIN32)

FIND_PATH(CMU1394_INCLUDE_DIR 1394Camera.h
  $ENV{1394CAMERACMU_DIR}/include
  $ENV{1394CAMERACMU_DIR}
  $ENV{1394CAMERACMU_HOME}/include
  $ENV{1394CAMERACMU_HOME}
  $ENV{EXTRA}/include
  $ENV{EXTRA}
)
#MESSAGE("DBG CMU1394_INCLUDE_DIR=${CMU1394_INCLUDE_DIR}")  
  
FIND_LIBRARY(CMU1394_LIBRARY
  NAMES 1394camera 1394Camera 1394camerad 1394Camerad
  PATHS 
  $ENV{1394CAMERACMU_DIR}/lib
  $ENV{1394CAMERACMU_DIR}
  $ENV{1394CAMERACMU_HOME}/lib
  $ENV{1394CAMERACMU_HOME}
  $ENV{EXTRA}/lib
  $ENV{EXTRA}
)
#MESSAGE("DBG CMU1394_LIBRARY=${CMU1394_LIBRARY}")
  
# --------------------------------

IF(CMU1394_LIBRARY)
  SET(CMU1394_LIBRARIES ${CMU1394_LIBRARY})
ENDIF(CMU1394_LIBRARY)

include(FindPackageHandleStandardArgs)
  # handle the QUIETLY and REQUIRED arguments and set SDH_FOUND to TRUE
  # if all listed variables are TRUE
find_package_handle_standard_args(CMU1394 DEFAULT_MSG 
                                  CMU1394_INCLUDE_DIR
                                  CMU1394_LIBRARIES)

MARK_AS_ADVANCED(
  CMU1394_INCLUDE_DIR
  CMU1394_LIBRARY
  CMU1394_LIBRARIES
)
ENDIF (NOT WIN32)




