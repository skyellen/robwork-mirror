
add_subdirectory(rwsim)
add_subdirectory(rwsimlibs)
add_subdirectory(sandbox)


#FIND_PACKAGE(Boost REQUIRED program_options)

#ADD_EXECUTABLE(RobWorkSimStudio main.cpp )
#SET(LIBS_TMP RWSimPluginStatic RWSimPluginStatic SimTaskPluginStatic  SimTaskVisPluginStatic
#rwsim_gui
#rwsim_ode
#ode
#rwsim
#${ROBWORKSIM_LIBRARIES}
#rws_plugin_lua
#rws_plugin_sensors
#rwstudio_imageview
#rwstudio_jog
#rwstudio_log
#rwstudio_lua
#rwstudio_planning
#rwstudio_playback
#rwstudio_propertyinspector
#rwstudio_propertyview
#rwstudio_sandbox
#rwstudio_treeview
#LuaEditorWindow
#qtpropertybrowser
#${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${Boost_LIBRARIES})
#MESSAGE("BUMBUM:  ${LIBS_TMP} ")
#TARGET_LINK_LIBRARIES(RobWorkSimStudio ${LIBS_TMP})
#ADD_DEPENDENCIES(RobWorkSimStudio rwstudio )
