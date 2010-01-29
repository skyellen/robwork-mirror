
# make sure that SDH can be found
FIND_PACKAGE(SDH)
IF(SDH_FOUND)
 MESSAGE(STATUS "SDH library found! SDHDriver will be build.")
 INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} )
 ADD_LIBRARY(rwhw_sdh SDHDriver.cpp SDHDriver.hpp)
 INSTALL(TARGETS rwhw_sdh DESTINATION ${RWHW_LIB_INSTALL_DIR})
ELSE()
 MESSAGE(STATUS "SDH library NOT found! SDHDriver will not be build.")
ENDIF()

