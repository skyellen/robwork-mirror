SET(SUBSYS_NAME rws_gtask)
set(SUBSYS_DESC "A sensor view plugin")
SET(SUBSYS_DEPS rws)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # if we want to use ui files add them here
    SET(UIS_FILES GTaskVisPlugin.ui )
    RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    
    # Standard cpp files to compile:
    SET(SrcFiles GTaskVisPlugin.cpp ${UIS_OUT_H})
    SET(SRC_FILES_HPP GTaskVisPlugin.hpp ${UIS_OUT_H})
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles gtask_resources.qrc)

    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwslibs/gtask" ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${ROBWORKSTUDIO_LIBRARIES} rw_task rw_proximitystrategies ${ROBWORK_LIBRARIES})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
