SET(SUBSYS_NAME rws_pointer)
set(SUBSYS_DESC "A pointer plugin")
SET(SUBSYS_DEPS rws)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # if we want to use ui files add them here
    SET(UIS_FILES PointerPlugin.ui )
    IF(RWS_USE_QT5)
      QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    ELSE()
      QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    ENDIF()
	#Need to add the current binary dir to the include directory because the generated source files are placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
	
    # Standard cpp files to compile:
    SET(SrcFiles PointerPlugin.cpp)
    SET(SRC_FILES_HPP PointerPlugin.hpp)
    
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
    
    IF(RWS_USE_QT5)
    	QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    	# Rcc the files:
    	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
    ELSE()
    # Header files to moc and compile:
    	QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    	# Rcc the files:
    	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
    ENDIF()
        
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    RW_ADD_INCLUDES(plugins "rwslibs/pointer" ${SRC_FILES_HPP}) 	
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
