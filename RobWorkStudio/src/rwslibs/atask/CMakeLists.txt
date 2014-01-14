SET(SUBSYS_NAME rws_atask)
set(SUBSYS_DESC "A plugin for visualization of assembly tasks and results.")
SET(SUBSYS_DEPS rws)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # if we want to use ui files add them here
    SET(UIS_FILES ATaskVisPlugin.ui )
    IF(RWS_USE_QT5)
      RWS_QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    ELSE()
      RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    ENDIF()
    
    # Standard cpp files to compile:
    SET(SrcFiles ATaskVisPlugin.cpp)
    SET(SRC_FILES_HPP ATaskVisPlugin.hpp)
    
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
    
    IF(RWS_USE_QT5)
    	QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    	# Rcc the files:
    	QT5_ADD_RESOURCES(RccSrcFiles atask_resources.qrc)
    ELSE()
    # Header files to moc and compile:
    	QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    	# Rcc the files:
    	QT4_ADD_RESOURCES(RccSrcFiles atask_resources.qrc)
    ENDIF()

    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    RW_ADD_INCLUDES(plugins "rwslibs/atask" ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${ROBWORKSTUDIO_LIBRARIES} rw_task ${ROBWORK_LIBRARIES})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
