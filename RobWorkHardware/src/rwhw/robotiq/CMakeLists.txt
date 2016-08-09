SET(SUBSYS_NAME robotiq )
SET(SUBSYS_DESC "driver for robotiq" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")    

    SET(SRC_CPP Robotiq.cpp Robotiq3.cpp Robotiq2.cpp)
    SET(SRC_HPP Robotiq.hpp Robotiq3.hpp Robotiq2.hpp)

    RW_ADD_LIBRARY(rwhw_robotiq robotiq ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(rwhw_robotiq ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(robotiq "rwhw/robotiq" ${SRC_HPP})
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_robotiq;")

	add_executable(robotiq3_test robotiq3SimpleTest.cpp )
	TARGET_LINK_LIBRARIES(robotiq3_test rwhw_robotiq ${ROBWORK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
	
	add_executable(robotiq2_test robotiq2SimpleTest.cpp )
	TARGET_LINK_LIBRARIES(robotiq2_test rwhw_robotiq ${ROBWORK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

ENDIF()
