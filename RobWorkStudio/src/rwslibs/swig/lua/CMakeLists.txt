SET(SUBSYS_NAME rws_lua )
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

	SET_SOURCE_FILES_PROPERTIES(../rws.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rws.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
    SWIG_ADD_MODULE(rws_lua lua ../rws.i Lua.cpp LuaState.cpp ../ScriptTypes.cpp)
    SWIG_LINK_LIBRARIES(rws_lua rws rws_rwstudioapp
                ${RWS_COMPONENT_LIBRARIES} 
                ${RWS_PLUGIN_LIBRARIES} 
                ${ROBWORKSTUDIO_LIBRARIES} 
                ${ROBWORK_LIBRARIES}
                ${BOOST_LIBRARIES})
    
    ADD_DEPENDENCIES(rws_lua rws_luaeditor rws_rwstudioapp)
    
    ADD_LIBRARY(rws_lua_s STATIC ${swig_generated_sources} ${swig_other_sources})
    LINK_LIBRARIES(rws_lua_s rws ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
    
    ADD_DEPENDENCIES(rws_lua_s rws LuaEditorWindow)
    
    SET(ENV{RWS_COMPONENT_LIBRARIES} "$ENV{RWS_COMPONENT_LIBRARIES}rws_lua_s;")
    #MESSAGE("${ROBWORKSTUDIO_LIBRARIES} \n ROBWORK ${ROBWORK_LIBRARIES}" )
    # this is used to indicate static linking to Visual Studio or mingw 
    	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(rws_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(rws_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()
    	
	
ENDIF()