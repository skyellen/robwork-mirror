
FIND_PACKAGE(SWIG 1.3)
IF( SWIG_FOUND )	

    SET(RWS_PLUGIN_LIBRARIES $ENV{RWS_PLUGIN_LIBRARIES})
    SET(RWS_COMPONENT_LIBRARIES $ENV{RWS_COMPONENT_LIBRARIES})
    

    INCLUDE(UseSWIG)
    SET(CMAKE_SWIG_FLAGS "")
    SET(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    IF( RWS_HAVE_LUA )
    	SET_SOURCE_FILES_PROPERTIES(rws.i PROPERTIES CPLUSPLUS ON)
    	SET_SOURCE_FILES_PROPERTIES(rws.i PROPERTIES SWIG_FLAGS "-includeall")
    
        SWIG_ADD_MODULE(rws_lua lua rws.i LuaState.cpp ScriptTypes.cpp)
        SWIG_LINK_LIBRARIES(rws_lua rws rws_rwstudioapp
                    ${RWS_COMPONENT_LIBRARIES} 
                    ${RWS_PLUGIN_LIBRARIES} 
                    ${ROBWORKSTUDIO_LIBRARIES} 
                    ${ROBWORK_LIBRARIES}
                    ${BOOST_LIBRARIES})
        
        ADD_DEPENDENCIES(rws_lua LuaEditorWindow rwscl_rwstudioapp)
        
        ADD_LIBRARY(rws_lua_s STATIC ${swig_generated_sources} ${swig_other_sources})
        LINK_LIBRARIES(rws_lua_s rwstudio ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
        ADD_DEPENDENCIES(rws_lua_s LuaEditorWindow)

        #MESSAGE("${ROBWORKSTUDIO_LIBRARIES} \n ROBWORK ${ROBWORK_LIBRARIES}" )
        # this is used to indicate static linking to Visual Studio or mingw 
	    	# this is used to indicate static linking to Visual Studio or mingw
    	IF (DEFINED MSVC) 
    	    SET_TARGET_PROPERTIES(rws_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    	ELSE()
    	    SET_TARGET_PROPERTIES(rws_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    	ENDIF()
    	
	ELSE()
	    MESSAGE(FATAL_ERROR "RobWorkStudio: SWIG not found, LUA disabled DISABLED!")
    ENDIF()

    #INCLUDE(${RW_ROOT}/cmake/RobWorkMacros.cmake)

    FIND_PACKAGE(PythonInterp)
    FIND_PACKAGE(PythonLibs)
    INCLUDE(CMakeDependentOption)
    CMAKE_DEPENDENT_OPTION(RWS_PYTHON_BINDINGS_ENABLED "Enable python bindings." 
                       ON "PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND" OFF)
    IF( RWS_PYTHON_BINDINGS_ENABLED )
        IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
            MESSAGE(STATUS "RobWorkStudio: Python bindings ENABLED!")
            INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
            SWIG_ADD_MODULE(rws_py python rws.i ScriptTypes.cpp)
            SWIG_LINK_LIBRARIES(rws_py rwstudio rws_rwstudioapp 
                ${RWS_COMPONENT_LIBRARIES} 
                ${RWS_PLUGIN_LIBRARIES} 
                ${ROBWORKSTUDIO_LIBRARIES} 
                ${ROBWORK_LIBRARIES} 
                ${PYTHON_LIBRARIES} )
            
        ELSE()
            MESSAGE(STATUS "RobWorkStudio: Python bindings DISABLED!")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "RobWorkStudio: Python bindings DISABLED")
    ENDIF()  
        
ENDIF()