SET(SUBSYS_NAME rw_simulation)
set(SUBSYS_DESC "Simulated sensors and controllers")
SET(SUBSYS_DEPS rw rw_opengl rw_control)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS glu opengl)
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    # The camera file and include path to use.
    
    SET(SRC_FILES
      ${RW_SENSORS_USER_FILES}
      ${CameraFile}
      ./camera/FrameGrabber.cpp
      ./camera/GLFrameGrabber.cpp
      ./camera/SimulatedCamera.cpp
      FrameGrabber25D.cpp
      GLFrameGrabber25D.cpp
      KinematicSimulator.cpp
      SimulatedController.cpp
      SimulatedSensor.cpp
      Simulator.cpp
      SimulatedScanner25D.cpp
      SimulatedScanner2D.cpp
    )

    SET(CAM_SRC_FILES_HPP
      ./camera/FrameGrabber.hpp
      ./camera/GLFrameGrabber.hpp
      ./camera/SimulatedCamera.hpp
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
    )
        
    RW_ADD_LIBRARY(${SUBSYS_NAME} simulation ${SRC_FILES} ${CAM_SRC_FILES_HPP} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(simulation "rwlibs/simulation" ${SRC_FILES_HPP}) 
    RW_ADD_INCLUDE_DIRS(simulation "rwlibs/simulation" camera)
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
