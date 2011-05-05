

FIND_PACKAGE(SwissRanger)
IF(SwissRanger_FOUND)
 MESSAGE(STATUS "SwissRanger library found! Swissranger driver will be build.")

 ADD_LIBRARY(rwhw_swissranger 
     SRCalibrationData.cpp SwissRanger.cpp
     SRCalibrationData.hpp SwissRanger.hpp SRConstants.hpp)
 INSTALL(TARGETS rwhw_swissranger DESTINATION ${RWHW_LIB_INSTALL_DIR})
 set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_swissranger;")
ELSE ()
 MESSAGE(STATUS "SwissRanger library NOT found! Swissranger driver will NOT be build.")
ENDIF()
