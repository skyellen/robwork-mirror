
FIND_PACKAGE(Lua51) # try setting ENV(LUA_DIR} if it fails

IF( LUA51_FOUND )
    INCLUDE_DIRECTORIES(${LUA_INCLUDE_DIR})
    # CMake does not allow a library and executable to have the same name, so we use
    # the name tolualib for the library.
    ADD_LIBRARY(
      tolua51
      src/lib/tolua_event.c
      src/lib/tolua_is.c
      src/lib/tolua_map.c
      src/lib/tolua_push.c
      src/lib/tolua_to.c
      #src/bin/tolua.c
      src/bin/toluabind.c)
      # src/bin/toluabind_default.c
    
    ADD_EXECUTABLE(tolua src/bin/tolua.c)
    TARGET_LINK_LIBRARIES(tolua tolua51 ${LUA_LIBRARIES} m)
    
    INSTALL(TARGETS tolua51 DESTINATION ${RW_LIB_INSTALL_DIR})
    INSTALL(TARGETS tolua DESTINATION ${RW_BIN_INSTALL_DIR})
ENDIF()