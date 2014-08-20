
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

# Files to moc and compile:
#SET(MocHeaderFiles ${SRC_FILES_HPP} )

#SET(MocSrcFiles )


IF (RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} ${SRC_FILES_CPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)   


ELSE()
	QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)	
QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.cpp" 
               "${CMAKE_CURRENT_BINARY_DIR}/src/qtpropertybrowser.moc" "${moc_flags}" "${moc_options}" "")
QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.cpp" 
	       "${CMAKE_CURRENT_BINARY_DIR}/src/qtpropertymanager.moc" "${moc_flags}" "${moc_options}" "")
QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.cpp"
	       "${CMAKE_CURRENT_BINARY_DIR}/src/qteditorfactory.moc" "${moc_flags}" "${moc_options}" "")
QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.cpp"
	       "${CMAKE_CURRENT_BINARY_DIR}/src/qttreepropertybrowser.moc" "${moc_flags}" "${moc_options}" "")


ENDIF()


set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowser.cxx")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertymanager.cxx;${CMAKE_CURRENT_BINARY_DIR}/qtpropertymanager.moc")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qteditorfactory.cxx;${CMAKE_CURRENT_BINARY_DIR}/qteditorfactory.moc")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtvariantproperty.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtvariantproperty.cxx")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qttreepropertybrowser.cxx;${CMAKE_CURRENT_BINARY_DIR}/qttreepropertybrowser.moc")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtbuttonpropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtbuttonpropertybrowser.cxx")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtgroupboxpropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtgroupboxpropertybrowser.cxx")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowserutils.cpp PROPERTIES 
	OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowserutils_p.cxx")
	       
					   

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
