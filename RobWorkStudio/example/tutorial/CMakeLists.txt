CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
# The name of the project.
PROJECT(SamplePluginApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT "${ROOT}/../../../RobWork")
SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
#Include default settings for constructing a robworkstudio dependent project
INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} )


#########################   From here we add the plugins 
QT4_WRAP_CPP(MocSrcFiles SamplePlugin.hpp)
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

  # The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE SamplePlugin.cpp ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
