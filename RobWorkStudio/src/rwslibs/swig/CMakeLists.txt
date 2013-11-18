
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
        MESSAGE(STATUS "RobWorkStudio: Lua bindings ENABLED!")
        SWIG_ADD_MODULE(rws_lua lua rws.i Lua.cpp LuaState.cpp ScriptTypes.cpp)
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
    	
	ELSE()
	    MESSAGE(FATAL_ERROR "RobWorkStudio: LUA DISABLED!")
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
            SWIG_ADD_MODULE(rws python rws.i ScriptTypes.cpp)
            SWIG_LINK_LIBRARIES(rws rws rws_rwstudioapp 
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
    
    FIND_PACKAGE(Java)
	
    FIND_PACKAGE(JNI)
    INCLUDE(CMakeDependentOption)
    CMAKE_DEPENDENT_OPTION(RW_JAVA_BINDINGS_ENABLED "Enable java bindings." 
                       ON "JAVA_FOUND AND JNI_FOUND" OFF)
    IF( RW_JAVA_BINDINGS_ENABLED )
		INCLUDE(UseJava)

        IF( JAVA_FOUND AND JNI_FOUND )
            MESSAGE(STATUS "RobWorkStudio: Java bindings ENABLED!")
            INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
            set(CMAKE_SWIG_FLAGS "-package" "dk.robwork")
            # Put java files in different directory suitable for JAR generation later on
            SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/dk/robwork)
            # SWIG
            SWIG_ADD_MODULE(rws_jni java rws.i)
            SWIG_LINK_LIBRARIES(rws_jni rws rws_rwstudioapp 
                ${RWS_COMPONENT_LIBRARIES} 
                ${RWS_PLUGIN_LIBRARIES} 
                ${ROBWORKSTUDIO_LIBRARIES}
                ${ROBWORK_LIBRARIES} )
	        ADD_DEPENDENCIES(rws_jni rws_luaeditor rws_rwstudioapp)
            # Force removal of previous Java compilation and source when interface file changes
            # This is required as types may be removed or change name
            # (in this case previous java classes would interfere with current compilation).
			ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
                COMMAND cmake -E remove_directory java_src
                COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
                DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/rws.i"
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Removing old Java source..."
            )
            ADD_CUSTOM_TARGET(CleanDep DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep )
            ADD_DEPENDENCIES(rws_jni CleanDep)
            # Compile java code and create JAR and Javadoc
			ADD_CUSTOM_COMMAND(TARGET rws_jni
				POST_BUILD
        		COMMAND cmake -E echo "Removing old Java compilation..."
                COMMAND cmake -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build"
            	COMMAND cmake -E remove_directory "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc"
        		COMMAND cmake -E echo "Copying Java source..."
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWS.java java_src/dk/robwork/LoaderRWS.java
        		COMMAND cmake -E echo "Compiling Java files..."
                COMMAND cmake -E make_directory java_build/dk/robwork
                COMMAND ${Java_JAVAC_EXECUTABLE} -cp ${RW_LIBS}/rw_java.jar -d ${CMAKE_CURRENT_BINARY_DIR}/java_build java_src/dk/robwork/*.java
                COMMAND cmake -E echo "Creating jar file..."
                COMMAND ${Java_JAR_EXECUTABLE} cvf ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/rws_java.jar  -C java_build .
    			COMMAND cmake -E echo "Creating Javadoc..."
            	COMMAND cmake -E make_directory ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
            	COMMAND ${Java_JAVADOC_EXECUTABLE}
            					-classpath ${RW_LIBS}/rw_java.jar
            					-d ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
            					-windowtitle "RobWorkStudio Java API Documentation"
            					-public
            					-sourcepath java_src dk.robwork
            					-linkoffline ${RW_LIBS}/javadoc ${RW_LIBS}/javadoc
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            )
            SET(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
        ELSE()
            MESSAGE(STATUS "RobWorkStudio: Java bindings DISABLED!")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "RobWorkStudio: Java bindings DISABLED")
    ENDIF()	

ENDIF()