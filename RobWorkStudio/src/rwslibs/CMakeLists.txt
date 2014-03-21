
#add_definitions(-DQT_PLUGIN)

if (DEFINED MSVC)
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D QT_NO_DEBUG")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -D QT_NO_DEBUG")
  SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -D QT_NO_DEBUG")
else ()
  add_definitions(-DQT_NO_DEBUG)
endif ()

SET(RWS_DEFAULT_LIB_TYPE SHARED)
IF( RWS_USE_STATIC_LINK_PLUGINS )
    SET(RWS_DEFAULT_LIB_TYPE STATIC)
ENDIF()

# The plugins:
add_subdirectory(atask)
add_subdirectory(gtask)
add_subdirectory(jog)
add_subdirectory(pointer)
add_subdirectory(log)
add_subdirectory(playback)
add_subdirectory(propertyview) 
add_subdirectory(treeview)
add_subdirectory(planning)
add_subdirectory(sensors)
add_subdirectory(luaeditor)
add_subdirectory(lua)
# depends on all above
add_subdirectory(rwstudioapp)
# this needs to be last
add_subdirectory(swig)


