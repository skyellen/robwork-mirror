INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

# to be able to include the generated ui header files
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 

################################################################
# next compile RWSimulatorPlugin
#
SET(SrcFiles RWSimulatorPlugin.cpp )

# Call the create_plugin macro for creating the plugin
IF( RWS_USE_QT5 )
QT5_WRAP_CPP(MocSrcFiles RWSimulatorPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
QT4_WRAP_CPP(MocSrcFiles RWSimulatorPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()

# Rcc the files:
SET(RccSrcFiles )

IF( RWS_USE_QT5 )
QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

ENDIF()
# The shared library to build:
ADD_LIBRARY(RWSimulatorPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(RWSimulatorPlugin ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})



################################################################
# next compile SDHPlugin
#
#include_directories(${ROOT}/../../RobWorkHardware/src/)
#link_directories(${ROOT}/../../RobWorkHardware/libs/Release/)
#link_directories(${ROOT}/../../RobWorkHardware/ext/sdh/libs/)
#link_directories("C:/Program Files/ESD/CAN/SDK/lib/vc/i386")

#SET(SrcFiles SDHPlugin.cpp )
#SET(MocSrcFiles SDHPlugin.cpp ) 
#SET(MocHeaderFiles SDHPlugin.hpp )
#SET(QrcFiles resources.qrc)

# Call the create_plugin macro for creating the plugin
#ADD_PLUGIN(SDHPlugin SrcFiles MocHeaderFiles QrcFiles)
#TARGET_LINK_LIBRARIES(SDHPlugin ${ROBWORKSIM_LIBRARIES} rwhw_sdh sdh ntcan.lib)




################################################################
# next compile SimUtilityPlugin
#
SET(SrcFiles SimUtilityPlugin.cpp )

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
IF(RWS_USE_QT5)
QT5_WRAP_CPP(MocSrcFiles SimUtilityPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
QT4_WRAP_CPP(MocSrcFiles SimUtilityPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()

# Rcc the files:
SET(RccSrcFiles )
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

# The shared library to build:
ADD_LIBRARY(SimUtilityPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimUtilityPlugin rwsim_gui ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})




################################################################
# next compile GraspTableGeneratorPlugin 
#
SET(UIS_OUT_H)
IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H GraspTableGeneratorPlugin.ui)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H GraspTableGeneratorPlugin.ui)
ENDIF()
SET(SrcFiles GraspTableGeneratorPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
IF(RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles GraspTableGeneratorPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles GraspTableGeneratorPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()

# Rcc the files:
SET(RccSrcFiles )
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

# The shared library to build:
ADD_LIBRARY(GraspTableGeneratorPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
TARGET_LINK_LIBRARIES(GraspTableGeneratorPlugin ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})




################################################################
# The RW Simulation 
#
SET(UIS_OUT_H)
IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H RWSimPlugin.ui)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H RWSimPlugin.ui)
ENDIF()
SET(SrcFiles RWSimPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
IF(RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles RWSimPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles RWSimPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()

# Rcc the files:
SET(RccSrcFiles )
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
ENDIF()
IF(RWSIM_HAVE_LUA)
    SET(RWSIM_LUA_LIB rwsim_lua_s)
    SET(RWS_LUAPL_LIBRARIES rws_luapl rws_sensors rws_planning rws_propertyview rws_playback rws_treeview rws_jog rws_log rws_rwstudioapp)
ENDIF()

ADD_LIBRARY(RWSimPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles} ${UIS_OUT_H})
#ADD_LIBRARY(RWSimPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(RWSimPlugin ${RWS_LUAPL_LIBRARIES} rwsim_gui ${RWSIM_LUA_LIB} ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
ADD_DEPENDENCIES( RWSimPlugin ${RWSIM_LUA_LIB} rwsim_gui)
#Message(" LIBS :  ${ROBWORKSIM_LIBRARIES} ")



################################################################
# The SimTaskPlugin
#
SET(UIS_OUT_H)
IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H SimTaskPlugin.ui)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H SimTaskPlugin.ui)
ENDIF()
SET(SrcFiles SimTaskPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
IF(RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles SimTaskPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles SimTaskPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()
# Rcc the files:
SET(RccSrcFiles )
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)
ENDIF()

ADD_LIBRARY(SimTaskPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
#ADD_LIBRARY(SimTaskPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTaskPlugin rwsim_gui rw_task ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})




################################################################
# The SimTaskVisPlugin -- has been moved to RobWorkStudio now named gtaskplugin
#
SET(UIS_OUT_H)
IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H SimTaskVisPlugin.ui)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H SimTaskVisPlugin.ui)
ENDIF()
#SET(SrcFiles SimTaskVisPlugin.cpp)
#set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
#SET(MocSrcFiles )
#QT4_WRAP_CPP(MocSrcFiles SimTaskVisPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
# Rcc the files:
#SET(RccSrcFiles )
#QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/SimTaskVisPlugin.qrc)

#ADD_LIBRARY(SimTaskVisPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles} ${UIS_OUT_H})
#ADD_LIBRARY(SimTaskVisPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
#TARGET_LINK_LIBRARIES(SimTaskVisPlugin rwsim_ode ode rwsim_gui rw_task ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})


################################################################
# next compile EngineTestPlugin
#
SET(UIS_OUT_H)
IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H EngineTestPlugin.ui)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H EngineTestPlugin.ui)
ENDIF()

SET(SrcFiles EngineTestPlugin.cpp )
#set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
IF(RWS_USE_QT5)
QT5_WRAP_CPP(MocSrcFiles EngineTestPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ELSE()
QT4_WRAP_CPP(MocSrcFiles EngineTestPlugin.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED OPTIONS -DBOOST_LEXICAL_CAST_INCLUDED)
ENDIF()

# Rcc the files:
SET(RccSrcFiles )
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

# The shared library to build:
ADD_LIBRARY(EngineTestPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles} ${UIS_OUT_H})
TARGET_LINK_LIBRARIES(EngineTestPlugin rwsim_test rwsim_gui ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
