CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(GraspPlugin)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWS_ROOT "$ENV{RWS_ROOT}")
SET(RWSIM_ROOT "$ENV{RWSIM_ROOT}")

SET(RobWork_DIR ${RW_ROOT}/cmake)
SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake)
SET(RobWorkSim_DIR ${RWSIM_ROOT}/cmake)

SET(CMAKE_MODULE_PATH ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(RobWork REQUIRED)
FIND_PACKAGE(RobWorkStudio REQUIRED)
FIND_PACKAGE(RobWorkSim REQUIRED)
FIND_PACKAGE(ODE)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS})
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE )

#########################
#From here we add the plugins
SET(SrcFiles
	TaskGenerator.cpp
	csgjs.cpp
	CSGConvert.cpp
	CSGModel.cpp
	JawPrimitive.cpp
	Gripper.cpp
	GripperXMLLoader.cpp
	GripperTaskSimulator.cpp
	TaskDescription.cpp
	XMLHelpers.cpp
)

SET(CreateGripperXmlSrc
	csgjs.cpp
	CSGConvert.cpp
	CSGModel.cpp
	JawPrimitive.cpp
	Gripper.cpp
	GripperXMLLoader.cpp
	TaskDescription.cpp
	XMLHelpers.cpp
)

SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles GraspPlugin.hpp DesignDialog.hpp)
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
	${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")

#MESSAGE(${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSIM_LIBRARIES})
#MESSAGE("!" ${ODE_LIBRARIES})
# The shared library to build
add_library( GraspPlugin MODULE GraspPlugin.cpp DesignDialog.cpp ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
add_executable( test test.cpp csgjs.cpp CSGConvert.cpp CSGModel.cpp JawPrimitive.cpp)
add_executable( create-gripper-xml createGripperXML.cpp ${CreateGripperXmlSrc})
target_link_libraries( GraspPlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSIM_LIBRARIES})
target_link_libraries( test ${ROBWORK_LIBRARIES})
target_link_libraries( create-gripper-xml ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSIM_LIBRARIES}
	GL GLU glut ode)

#MESSAGE(${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSIM_LIBRARIES})
