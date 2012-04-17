INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

################################################################
# next compile RWSimulatorPlugin
#
SET(SrcFiles RWSimulatorPlugin.cpp )

# Call the create_plugin macro for creating the plugin
QT4_WRAP_CPP(MocSrcFiles RWSimulatorPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# The shared library to build:
ADD_LIBRARY(RWSimulatorPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(RWSimulatorPlugin ${ROBWORKSIM_LIBRARIES})



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
QT4_WRAP_CPP(MocSrcFiles SimUtilityPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# The shared library to build:
ADD_LIBRARY(SimUtilityPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimUtilityPlugin ${ROBWORKSIM_LIBRARIES})




################################################################
# next compile GraspTableGeneratorPlugin 
#
RWS_QT4_WRAP_UI(UIS_OUT_H GraspTableGeneratorPlugin.ui)
SET(SrcFiles GraspTableGeneratorPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles GraspTableGeneratorPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# The shared library to build:
ADD_LIBRARY(GraspTableGeneratorPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(GraspTableGeneratorPlugin ${ROBWORKSIM_LIBRARIES})




################################################################
# The RW Simulation 
#
RWS_QT4_WRAP_UI(UIS_OUT_H RWSimPlugin.ui)
SET(SrcFiles RWSimPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles RWSimPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)
IF(RWSIM_HAVE_LUA)
    SET(RWSIM_LUA_LIB rwsim_lua_s)
ENDIF()

ADD_LIBRARY(RWSimPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
#ADD_LIBRARY(RWSimPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(RWSimPlugin ${RWSIM_LUA_LIB} rwsim_ode ode rwsim_gui ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
ADD_DEPENDENCIES( RWSimPlugin ${RWSIM_LUA_LIB} rwsim_ode rwsim_gui)
#Message(" LIBS :  ${ROBWORKSIM_LIBRARIES} ")



################################################################
# The SimTaskPlugin
#
RWS_QT4_WRAP_UI(UIS_OUT_H SimTaskPlugin.ui)
SET(SrcFiles SimTaskPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles SimTaskPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)

ADD_LIBRARY(SimTaskPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
#ADD_LIBRARY(SimTaskPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTaskPlugin rwsim_ode ode rwsim_gui rw_task ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})




################################################################
# The SimTaskVisPlugin
#
RWS_QT4_WRAP_UI(UIS_OUT_H SimTaskVisPlugin.ui)
SET(SrcFiles SimTaskVisPlugin.cpp)
set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Call the create_plugin macro for creating the plugin
SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles SimTaskVisPlugin.hpp)
# Rcc the files:
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles ${RWSIM_ROOT}/src/gfx/SimTaskVisPlugin.qrc)

ADD_LIBRARY(SimTaskVisPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
#ADD_LIBRARY(SimTaskVisPluginStatic STATIC ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTaskVisPlugin rwsim_ode ode rwsim_gui rw_task ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
