SET(SUBSYS_NAME rw_calibration)
set(SUBSYS_DESC "Serial device calibration")
SET(SUBSYS_DEPS rw ${QT_LIBRARIES})

IF (RW_HAVE_EIGEN AND RW_HAVE_QT)
	set(build TRUE)
ELSE()
	set(build FALSE)
ENDIF()

RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build=TRUE)
    SET(SRC_FILES
		Pose6D.cpp
		SerialDeviceCalibration.cpp
    )
    
    SET(SRC_FILES_HPP
    	Pose6D.hpp
    	RPY.hpp
    	SerialDeviceCalibration.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} calibration ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(calibration "rwlibs/calibration" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
