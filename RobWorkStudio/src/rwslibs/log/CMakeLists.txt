SET(SUBSYS_NAME rws_log)
set(SUBSYS_DESC "A logging plugin")
SET(SUBSYS_DEPS rws)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles ShowLog.cpp)
    SET(SRC_FILES_HPP ShowLog.cpp)
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ShowLog.hpp)
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
            
    # Set extra linker flags for apple macos computers    
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwslibs/log" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()