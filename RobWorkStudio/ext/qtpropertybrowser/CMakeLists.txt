
SET(TargetName qtpropertybrowser)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src)

SET(MYROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)




set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ADD_DEFINITIONS("-DQT_DISABLE_DEPRECATED_BEFORE=0")
ADD_DEFINITIONS("-DQT_NO_DEBUG")
ADD_DEFINITIONS("-DQT_WIDGETS_LIB")
ADD_DEFINITIONS("-DQT_GUI_LIB")
ADD_DEFINITIONS("-DQT_CORE_LIB")

# Standard files to compile:
SET(SRC_FILES_CPP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtvariantproperty.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtbuttonpropertybrowser.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtgroupboxpropertybrowser.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowserutils.cpp
  #  ${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowserutils_p.cpp
  ${UIS_OUT_H})

SET(SRC_FILES_HPP		
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtvariantproperty.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtbuttonpropertybrowser.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtgroupboxpropertybrowser.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowserutils_p.h)


# Resource files to rcc and compile:
IF (RWS_USE_QT5)
	QT5_ADD_RESOURCES(ResourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.qrc)
ELSE()
	QT4_ADD_RESOURCES(ResourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.qrc)
ENDIF()

# We create a static library named qtpropertybrowser that all plugins should link to. Our
# RobWorkStudio executable links to this library also.
ADD_LIBRARY(${TargetName} ${SRC_FILES_CPP} ${ResourceFiles})

# Install stuff
INSTALL(TARGETS ${TargetName} DESTINATION "${LIB_INSTALL_DIR}")
INSTALL(DIRECTORY "${RWS_ROOT}/ext/qtpropertybrowser/"
		DESTINATION "${EXT_INSTALL_DIR}/qtpropertybrowser/"
		FILES_MATCHING
		PATTERN ".svn" EXCLUDE
		PATTERN "*.cpp" EXCLUDE
		PATTERN "projects" EXCLUDE
		PATTERN "examples" EXCLUDE
		PATTERN "doc" EXCLUDE
		PATTERN "buildlib" EXCLUDE
		)
