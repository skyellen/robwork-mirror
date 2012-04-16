#FIND_PACKAGE(Tolua++)

FIND_PACKAGE(SWIG 1.3 QUIET)
#MESSAGE(STATUS "${SWIG_FOUND}")
#MESSAGE(STATUS "${SWIG_DIR}")
#MESSAGE(STATUS "${SWIG_EXECUTABLE}")
#MESSAGE(STATUS "${SWIG_VERSION}")

IF( SWIG_FOUND ) 
    SET(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(rwsim.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(rwsim.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
    SWIG_ADD_MODULE(rwsim_lua lua rwsim.i ScriptTypes.cpp Lua.cpp)
	SWIG_LINK_LIBRARIES(rwsim_lua rw ${ROBWORKSIM_LIBRARIES} ${ROBWORK_LIBRARIES})

    ADD_LIBRARY(rwsim_lua_s STATIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
    LINK_LIBRARIES(rwsim_lua_s rw ${ROBWORKSIM_LIBRARIES} ${ROBWORK_LIBRARIES})
	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(rwsim_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(rwsim_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()
	
ENDIF()
