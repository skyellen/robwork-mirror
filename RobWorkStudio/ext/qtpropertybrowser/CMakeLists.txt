
SET(TargetName qtpropertybrowser)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src)

SET(MYROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)


# helper macro to set up a moc rule
MACRO (RWS_QT4_CREATE_MOC_COMMAND infile outfile moc_flags moc_options)
  # For Windows, create a parameters file to work around command line length limit
  IF (WIN32)
    # Pass the parameters in a file.  Set the working directory to
    # be that containing the parameters file and reference it by
    # just the file name.  This is necessary because the moc tool on
    # MinGW builds does not seem to handle spaces in the path to the
    # file given with the @ syntax.
    GET_FILENAME_COMPONENT(_moc_outfile_name "${outfile}" NAME)
    GET_FILENAME_COMPONENT(_moc_outfile_dir "${outfile}" PATH)
    IF(_moc_outfile_dir)
      SET(_moc_working_dir WORKING_DIRECTORY ${_moc_outfile_dir})
    ENDIF(_moc_outfile_dir)
    SET (_moc_parameters_file ${outfile}_parameters)
    SET (_moc_parameters ${moc_flags} ${moc_options} -o "${outfile}" "${infile}")
    FILE (REMOVE ${_moc_parameters_file})
    FOREACH(arg ${_moc_parameters})
      FILE (APPEND ${_moc_parameters_file} "${arg}\n")
    ENDFOREACH(arg)
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
                       COMMAND ${QT_MOC_EXECUTABLE} @${_moc_outfile_name}_parameters
                       #DEPENDS ${infile}
                       ${_moc_working_dir}
                       VERBATIM)
  ELSE (WIN32)
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
                       COMMAND ${QT_MOC_EXECUTABLE}
                       ARGS ${moc_flags} ${moc_options} -o ${outfile} ${infile}
                       #DEPENDS ${infile}
                       )
  ENDIF (WIN32)
ENDMACRO (RWS_QT4_CREATE_MOC_COMMAND)


# macro used to create the names of output files preserving relative dirs
MACRO (RWS_QT4_MAKE_OUTPUT_FILE infile prefix ext outfile )
  STRING(LENGTH ${CMAKE_CURRENT_BINARY_DIR} _binlength)
  STRING(LENGTH ${infile} _infileLength)
  SET(_checkinfile ${CMAKE_CURRENT_SOURCE_DIR})
  IF(_infileLength GREATER _binlength)
    STRING(SUBSTRING "${infile}" 0 ${_binlength} _checkinfile)
    IF(_checkinfile STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
      FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_BINARY_DIR} ${infile})
    ELSE(_checkinfile STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
      FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${infile})
    ENDIF(_checkinfile STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  ELSE(_infileLength GREATER _binlength)
    FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${infile})
  ENDIF(_infileLength GREATER _binlength)
  IF(WIN32 AND rel MATCHES "^[a-zA-Z]:") # absolute path 
    STRING(REGEX REPLACE "^([a-zA-Z]):(.*)$" "\\1_\\2" rel "${rel}")
  ENDIF(WIN32 AND rel MATCHES "^[a-zA-Z]:") 
  SET(_outfile "${CMAKE_CURRENT_BINARY_DIR}/${rel}")
  STRING(REPLACE ".." "__" _outfile ${_outfile})
  GET_FILENAME_COMPONENT(outpath ${_outfile} PATH)
  GET_FILENAME_COMPONENT(_outfile ${_outfile} NAME_WE)
  FILE(MAKE_DIRECTORY ${outpath})
  SET(${outfile} ${outpath}/${prefix}${_outfile}.${ext})
ENDMACRO (RWS_QT4_MAKE_OUTPUT_FILE )

MACRO (RWS_QT4_WRAP_CPP outfiles )
  # get include dirs
  QT4_GET_MOC_FLAGS(moc_flags)
  QT4_EXTRACT_OPTIONS(moc_files moc_options ${ARGN})

  FOREACH (it ${moc_files})
    GET_FILENAME_COMPONENT(it ${it} ABSOLUTE)
    RWS_QT4_MAKE_OUTPUT_FILE(${it} moc_ cpp outfile)
    QT4_CREATE_MOC_COMMAND(${it} ${outfile} "${moc_flags}" "${moc_options}")
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH(it)

ENDMACRO (RWS_QT4_WRAP_CPP)

MACRO (RWS_QT5_WRAP_CPP outfiles )
  # get include dirs
  QT5_GET_MOC_FLAGS(moc_flags)
  QT5_EXTRACT_OPTIONS(moc_files moc_options ${ARGN})

  FOREACH (it ${moc_files})
    GET_FILENAME_COMPONENT(it ${it} ABSOLUTE)
    RWS_QT4_MAKE_OUTPUT_FILE(${it} moc_ cpp outfile)
    QT5_CREATE_MOC_COMMAND(${it} ${outfile} "${moc_flags}" "${moc_options}")
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH(it)

ENDMACRO (RWS_QT5_WRAP_CPP)

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
    ${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowserutils_p.cpp
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
SET(MocHeaderFiles ${SRC_FILES_HPP} )

SET(MocSrcFiles )

IF (RWS_USE_QT5)
	RWS_QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
ELSE()
	RWS_QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
ENDIF()

	RWS_QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.cpp"
                       "${CMAKE_CURRENT_BINARY_DIR}/src/qttreepropertybrowser.moc" "${moc_flags}" "${moc_options}")
RWS_QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.cpp" 
                       "${CMAKE_CURRENT_BINARY_DIR}/src/qtpropertymanager.moc" "${moc_flags}" "${moc_options}")
RWS_QT4_CREATE_MOC_COMMAND("${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.cpp" 
                       "${CMAKE_CURRENT_BINARY_DIR}/src/qteditorfactory.moc" "${moc_flags}" "${moc_options}")

set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowser.cpp")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertymanager.cpp;${CMAKE_CURRENT_BINARY_DIR}/src/qtpropertymanager.moc")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qteditorfactory.cpp;${CMAKE_CURRENT_BINARY_DIR}/src/qteditorfactory.moc")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtvariantproperty.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtvariantproperty.cpp")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qttreepropertybrowser.cpp;${CMAKE_CURRENT_BINARY_DIR}/src/qttreepropertybrowser.moc")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtbuttonpropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtbuttonpropertybrowser.cpp")
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtgroupboxpropertybrowser.cpp PROPERTIES 
    OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtgroupboxpropertybrowser.cpp")
#set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowserutils.cpp PROPERTIES OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowserutils.cpp")

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
