
IF( RWS_HAVE_LUA )
       
    FIND_PACKAGE(SWIG 1.3 QUIET)
    IF( SWIG_FOUND )
    	
    	INCLUDE(UseSWIG)
    	SET(CMAKE_SWIG_FLAGS "")
    
    	SET_SOURCE_FILES_PROPERTIES(rws.i PROPERTIES CPLUSPLUS ON)
    	SET_SOURCE_FILES_PROPERTIES(rws.i PROPERTIES SWIG_FLAGS "-includeall")
    
        SWIG_ADD_MODULE(rwslua lua rws.i LuaState.cpp ScriptTypes.cpp)
        SWIG_LINK_LIBRARIES(rwslua rwstudio 
                    ${RWS_COMPONENT_LIBRARIES} 
                    ${RWS_PLUGIN_LIBRARIES} 
                    ${ROBWORKSTUDIO_LIBRARIES} 
                    ${ROBWORK_LIBRARIES})
        
        ADD_DEPENDENCIES(rwslua LuaEditorWindow)
        
        ADD_LIBRARY(rwstudio_lua STATIC ${swig_generated_sources} ${swig_other_sources})
        LINK_LIBRARIES(rwstudio_lua rwstudio ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})

        ADD_DEPENDENCIES(rwstudio_lua LuaEditorWindow)

        #MESSAGE("${ROBWORKSTUDIO_LIBRARIES} \n ROBWORK ${ROBWORK_LIBRARIES}" )
        # this is used to indicate static linking to Visual Studio or mingw 
	    	# this is used to indicate static linking to Visual Studio or mingw
    	IF (DEFINED MSVC) 
    	    SET_TARGET_PROPERTIES(rwstudio_lua PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    	ELSE()
    	    SET_TARGET_PROPERTIES(rwstudio_lua PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    	ENDIF()
	    
	    
	
    	#FIND_PACKAGE(PythonInterp)
    	#FIND_PACKAGE(PythonLibs)
    	
    	#IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
    	#    MESSAGE(STATUS "RobWorkStudio: Python interface generation ENABLED!")
    	#    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
    	#    SWIG_ADD_MODULE(rws python rws.i)
    	#    SWIG_LINK_LIBRARIES(rws rwstudio ${RWS_COMPONENT_LIBRARIES} ${RWS_PLUGIN_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${PYTHON_LIBRARIES} )
    	#ELSE()
    	#    MESSAGE(STATUS "RobWorkStudio: Python interface generation DISABLED!")
    	#ENDIF()  
    	
	ELSE()
	    MESSAGE(FATAL_ERROR "RobWorkStudio: SWIG not found, LUA disabled DISABLED!")
    ENDIF()

ENDIF()
