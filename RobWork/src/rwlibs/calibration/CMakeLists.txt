
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
	SET(SRC_FILES
		nlls/NLLSIterationLog.cpp
		nlls/NLLSNewtonSolver.cpp
		nlls/NLLSSystem.cpp
		xml/XmlCalibrationLoader.cpp
        xml/XmlCalibrationSaver.cpp
        xml/XmlMeasurementFile.cpp
		JacobianBase.cpp
		Calibration.cpp
		CalibrationBase.cpp
		CalibrationParameter.cpp
		CalibrationParameterSet.cpp
		DHLinkCalibration.cpp
		DHLinkJacobian.cpp
		JointEncoderCalibration.cpp
		JointEncoderJacobian.cpp
		FixedFrameCalibration.cpp
		FixedFrameJacobian.cpp
		SerialDeviceCalibration.cpp
		SerialDeviceCalibrator.cpp
		SerialDeviceJacobian.cpp
		SerialDevicePoseMeasurement.cpp
	)
    
	SET(SRC_FILES_HPP
		nlls/NLLSIterationLog.hpp
		nlls/NLLSNewtonSolver.hpp
		nlls/NLLSSolver.hpp
		nlls/NLLSSystem.hpp
		xml/XmlCalibrationLoader.hpp
        xml/XmlCalibrationSaver.hpp
        xml/XmlMeasurementFile.hpp
		JacobianBase.hpp
		Calibration.hpp
		CalibrationBase.hpp
		CalibrationParameter.hpp
		CalibrationParameterSet.hpp
		CompositeCalibration.hpp
		CompositeJacobian.hpp
		DHLinkCalibration.hpp
		DHLinkJacobian.hpp
		EigenTransformPlugin.hpp
		FixedFrameCalibration.hpp
		FixedFrameJacobian.hpp
		Jacobian.hpp
		JointEncoderCalibration.hpp
		JointEncoderJacobian.hpp
		SerialDeviceCalibration.hpp
		SerialDeviceCalibrator.hpp
		SerialDeviceJacobian.hpp
		SerialDevicePoseMeasurement.hpp
	)
    
	RW_ADD_LIBRARY( ${SUBSYS_NAME} calibration ${SRC_FILES} ${SRC_FILES_HPP} )
	RW_ADD_INCLUDES( calibration "rwlibs/calibration" ${SRC_FILES_HPP} ) 
	TARGET_LINK_LIBRARIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )
	ADD_DEPENDENCIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )
ENDIF()
