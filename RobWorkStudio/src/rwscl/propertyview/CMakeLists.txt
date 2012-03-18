SET(SUBSYS_NAME rwscl_propertyview)
set(SUBSYS_DESC "A plugin for editing property maps")
SET(SUBSYS_DEPS rwstudio) # also depends on qtpropertyviewer

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles PropertyView.cpp)
    SET(SRC_FILES_HPP PropertyView.hpp)
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
        
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwscl/propertyview" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
