SET(SUBSYS_NAME rw_calibration )
SET(SUBSYS_DESC "Calibration of devices" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
MESSAGE("CALIBRATION SET FILES")
	SET(SRC_FILES
		xml/XmlCalibrationLoader.cpp
        xml/XmlCalibrationSaver.cpp        
		Calibration.cpp
		FixedFrameCalibration.cpp
		WorkCellCalibration.cpp
		
	)
    
	SET(SRC_FILES_HPP
		xml/XmlCalibrationLoader.hpp
        xml/XmlCalibrationSaver.hpp
		Calibration.hpp
		FixedFrameCalibration.hpp
		WorkCellCalibration.hpp		
	)
    
	RW_ADD_LIBRARY( ${SUBSYS_NAME} calibration ${SRC_FILES} ${SRC_FILES_HPP} )
	RW_ADD_INCLUDES( calibration "rwlibs/calibration" ${SRC_FILES_HPP} ) 
	TARGET_LINK_LIBRARIES( ${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} )
	ADD_DEPENDENCIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()
