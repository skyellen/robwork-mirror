SET(SUBSYS_NAME rws_sensors)
set(SUBSYS_DESC "A sensor view plugin")
SET(SUBSYS_DEPS rws)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # if we want to use ui files add them here
    SET(UIS_FILES SensorsPlugin.ui )
    RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    
    # Standard cpp files to compile:
    SET(SrcFiles Sensors.cpp SensorView.cpp)
    SET(SRC_FILES_HPP Sensors.hpp SensorView.hpp)
    
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
        
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    RW_ADD_INCLUDES(plugins "rwslibs/sensors" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
