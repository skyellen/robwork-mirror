CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
# The name of the project.
PROJECT(SamplePluginApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT "${ROOT}/../../../RobWork")
SET(RWS_ROOT "${ROOT}/../../../RobWorkStudio")

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)

SET(RobWorkStudio_DIR "${RWS_ROOT}/cmake") 
FIND_PACKAGE(RobWorkStudio)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} )

#########################   From here we add the plugins 
QT4_WRAP_CPP(MocSrcFiles SamplePlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

  # The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE SamplePlugin.cpp ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
