SET(SUBSYS_NAME rw_simulation)
set(SUBSYS_DESC "Simulated sensors and controllers")
SET(SUBSYS_DEPS rw rw_control)

SET(build TRUE)
SET(DEFAULT TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS glu opengl)
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    
    SET(SRC_FILES
      ${RW_SENSORS_USER_FILES}
      FrameGrabber.cpp
      FrameGrabber25D.cpp
      GLFrameGrabber.cpp
      SimulatedCamera.cpp
      SimulatedKinect.cpp
      GLFrameGrabber25D.cpp
      KinematicSimulator.cpp
      SimulatedController.cpp
      SimulatedSensor.cpp
      Simulator.cpp
      SimulatedScanner25D.cpp
      SimulatedScanner2D.cpp
    )

    SET(CAM_SRC_FILES_HPP
      FrameGrabber.hpp
      GLFrameGrabber.hpp
      SimulatedCamera.hpp
    )
        
    SET(SRC_FILES_HPP
      FrameGrabber25D.hpp
      GLFrameGrabber25D.hpp
      KinematicSimulator.hpp
      SimulatedController.hpp
      SimulatedSensor.hpp
      Simulator.hpp
      SimulatedScanner25D.hpp
      SimulatedScanner2D.hpp
      SimulatedKinect.hpp
    )
        
    RW_ADD_LIBRARY(${SUBSYS_NAME} simulation ${SRC_FILES} ${CAM_SRC_FILES_HPP} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(simulation "rwlibs/simulation" ${SRC_FILES_HPP}) 
    RW_ADD_INCLUDE_DIRS(simulation "rwlibs/simulation" camera)
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()
