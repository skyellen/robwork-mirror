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
		nlls/NLLSIterationLog.cpp
		nlls/NLLSNewtonSolver.cpp
		nlls/NLLSSystem.cpp
		xml/XmlCalibrationLoader.cpp
        xml/XmlCalibrationSaver.cpp        
		xml/XMLCalibrationMeasurementFile.cpp
		JacobianBase.cpp
		Calibration.cpp
		CalibrationBase.cpp
		CalibrationParameter.cpp
		CalibrationParameterSet.cpp
		JointEncoderCalibration.cpp
		JointEncoderJacobian.cpp 
		FixedFrameCalibration.cpp
		FixedFrameJacobian.cpp
		WorkCellCalibration.cpp
		WorkCellCalibrator.cpp
		WorkCellExtrinsicCalibrator.cpp
		WorkCellJacobian.cpp
		ParallelAxisDHCalibration.cpp
		ParallelAxisDHJacobian.cpp
		CalibrationMeasurement.cpp
		CalibrationUtils.cpp
		
	)
    
	SET(SRC_FILES_HPP
		nlls/NLLSIterationLog.hpp
		nlls/NLLSNewtonSolver.hpp
		nlls/NLLSSolver.hpp
		nlls/NLLSSystem.hpp
		xml/XmlCalibrationLoader.hpp
        xml/XmlCalibrationSaver.hpp
		xml/XMLCalibrationMeasurementFile.hpp
		JacobianBase.hpp
		Calibration.hpp
		CalibrationBase.hpp
		CalibrationParameter.hpp
		CalibrationParameterSet.hpp
		CompositeCalibration.hpp
		CompositeJacobian.hpp
		FixedFrameCalibration.hpp
		FixedFrameJacobian.hpp
		Jacobian.hpp
		JointEncoderCalibration.hpp
		JointEncoderJacobian.hpp
		WorkCellCalibration.hpp
		WorkCellCalibrator.hpp
		WorkCellExtrinsicCalibrator.hpp
		WorkCellJacobian.hpp
		ParallelAxisDHCalibration.hpp
		ParallelAxisDHJacobian.hpp
		CalibrationMeasurement.hpp
		CalibrationUtils.hpp
		
	)
    
	RW_ADD_LIBRARY( ${SUBSYS_NAME} calibration ${SRC_FILES} ${SRC_FILES_HPP} )
	RW_ADD_INCLUDES( calibration "rwlibs/calibration" ${SRC_FILES_HPP} ) 
	TARGET_LINK_LIBRARIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )
	ADD_DEPENDENCIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )
ENDIF()
