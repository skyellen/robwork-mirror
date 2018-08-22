SET(SUBSYS_NAME rwsim_luai )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from lua." )
SET(SUBSYS_DEPS rw )

# make sure that SDH can be found
SET(build TRUE)

FIND_PACKAGE(SWIG 1.3 QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   SET(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(../rwsim.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rwsim.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(rwsim_lua TYPE MODULE LANGUAGE lua SOURCES ../rwsim.i ../ScriptTypes.cpp Lua.cpp)
	ELSE()
		SWIG_ADD_MODULE(rwsim_lua lua ../rwsim.i ../ScriptTypes.cpp Lua.cpp)
	ENDIF()
	SWIG_LINK_LIBRARIES(rwsim_lua ${RWSIM_ODE_LIBRARY} rwsim ${ROBWORK_LIBRARIES})
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(${SWIG_MODULE_rwsim_lua_REAL_NAME} PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()

    IF ((CMAKE_VERSION VERSION_GREATER 3.12.0) OR (CMAKE_VERSION VERSION_EQUAL 3.12.0))
        SWIG_ADD_LIBRARY(rwsim_lua_s TYPE STATIC LANGUAGE lua SOURCES ../rwsim.i ../ScriptTypes.cpp Lua.cpp)
    ELSE()
        ADD_LIBRARY(rwsim_lua_s STATIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
        ADD_DEPENDENCIES(rwsim_lua_s rwsim_lua) # avoid using the source files before they have been generated
    ENDIF()
	TARGET_LINK_LIBRARIES(rwsim_lua_s ${RWSIM_ODE_LIBRARY} rwsim ${ROBWORK_LIBRARIES})
	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(rwsim_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(rwsim_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()
	
	ADD_LIBRARY(rwsimlua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)
    TARGET_LINK_LIBRARIES(rwsimlua_plugin.rwplugin rwsim_lua_s ${ROBWORK_LIBRARIES})
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(rwsimlua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()
	
	
ENDIF()