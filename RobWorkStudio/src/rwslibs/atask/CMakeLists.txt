SET(SUBSYS_NAME rws_atask)
set(SUBSYS_DESC "A plugin for visualization of assembly tasks and results.")
SET(SUBSYS_DEPS rws)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # if we want to use ui files add them here
    IF(RWS_USE_QT5)
      QT5_WRAP_UI(UIS_OUT_H_ATaskVisPlugin ATaskVisPlugin.ui)
      QT5_WRAP_UI(UIS_OUT_H_StrategyLibraryDialog StrategyLibraryDialog.ui)
      QT5_WRAP_UI(UIS_OUT_H_TaskSetupDialog TaskSetupDialog.ui)
    ELSE()
      QT4_WRAP_UI(UIS_OUT_H_ATaskVisPlugin ATaskVisPlugin.ui)
      QT4_WRAP_UI(UIS_OUT_H_StrategyLibraryDialog StrategyLibraryDialog.ui)
      QT4_WRAP_UI(UIS_OUT_H_TaskSetupDialog TaskSetupDialog.ui)
    ENDIF()
    SET(UIS_OUT_H ${UIS_OUT_H_ATaskVisPlugin} ${UIS_OUT_H_StrategyLibraryDialog} ${UIS_OUT_H_TaskSetupDialog})
	#Need to add the current binary dir to the include directory because the generated source files are placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    # Standard cpp files to compile:
    SET(SrcFiles ATaskVisPlugin.cpp StrategyLibraryDialog.cpp TaskSetupDialog.cpp)
    SET(SRC_FILES_HPP ATaskVisPlugin.hpp StrategyLibraryDialog.hpp TaskSetupDialog.hpp)
    
    set_source_files_properties(ATaskVisPlugin.cpp PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H_ATaskVisPlugin}")
    set_source_files_properties(StrategyLibraryDialog.cpp PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H_StrategyLibraryDialog}")
    set_source_files_properties(TaskSetupDialog.cpp PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H_TaskSetupDialog}")
    
    IF(RWS_USE_QT5)
    	QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
    	# Rcc the files:
    	QT5_ADD_RESOURCES(RccSrcFiles atask_resources.qrc)
    ELSE()
    # Header files to moc and compile:
    	QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
    	# Rcc the files:
    	QT4_ADD_RESOURCES(RccSrcFiles atask_resources.qrc)
    ENDIF()
    
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    RW_ADD_INCLUDES(plugins "rwslibs/atask" ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
