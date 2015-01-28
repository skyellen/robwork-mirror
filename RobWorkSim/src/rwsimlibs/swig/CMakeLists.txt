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
	
	FIND_PACKAGE(PythonInterp)
    FIND_PACKAGE(PythonLibs)
    INCLUDE(CMakeDependentOption)
    CMAKE_DEPENDENT_OPTION(RWSIM_PYTHON_BINDINGS_ENABLED "Enable python bindings." 
                       ON "PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND" OFF)
    IF( RWSIM_PYTHON_BINDINGS_ENABLED )
        IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
            MESSAGE(STATUS "RobWorkSim: Python bindings ENABLED!")
            INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
            
            SET(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
            SWIG_ADD_MODULE(rwsim python rwsim.i ScriptTypes.cpp)
            SWIG_LINK_LIBRARIES(rwsim rwsim
                ${ROBWORK_LIBRARIES} 
                ${PYTHON_LIBRARIES} )
  
        ELSE()
            MESSAGE(STATUS "RobWorkSim: Python bindings DISABLED!")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "RobWorkSim: Python bindings DISABLED")
    ENDIF()
    
    FIND_PACKAGE(Java)
	
    FIND_PACKAGE(JNI)
    INCLUDE(CMakeDependentOption)
    CMAKE_DEPENDENT_OPTION(RW_JAVA_BINDINGS_ENABLED "Enable java bindings." 
                       ON "JAVA_FOUND AND JNI_FOUND" OFF)
    IF( RW_JAVA_BINDINGS_ENABLED )
		INCLUDE(UseJava)

        IF( JAVA_FOUND AND JNI_FOUND )
            MESSAGE(STATUS "RobWorkSim: Java bindings ENABLED!")
            INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
            set(CMAKE_SWIG_FLAGS "-package" "dk.robwork")
            # Put java files in different directory suitable for JAR generation later on
            SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/dk/robwork)
            # SWIG
            SWIG_ADD_MODULE(rwsim_jni java rwsim.i ThreadSimulatorStepCallbackEnv.cpp)
            SWIG_LINK_LIBRARIES(rwsim_jni rwsim ${ROBWORKSIM_LIBRARIES} ${ROBWORK_LIBRARIES} )
            # Force removal of previous Java compilation and source when interface file changes
            # This is required as types may be removed or change name
            # (in this case previous java classes would interfere with current compilation).
			ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
                COMMAND cmake -E remove_directory java_src
                COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
                DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/rwsim.i"
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Removing old Java source..."
            )
            ADD_CUSTOM_TARGET(CleanDep DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep )
            ADD_DEPENDENCIES(rwsim_jni CleanDep)
            # Compile java code and create JAR and Javadoc
			ADD_CUSTOM_COMMAND(TARGET rwsim_jni
				POST_BUILD
        		COMMAND cmake -E echo "Removing old Java compilation..."
                COMMAND cmake -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build"
            	COMMAND cmake -E remove_directory "${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc"
        		COMMAND cmake -E echo "Copying Java source..."
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWSim.java java_src/dk/robwork/LoaderRWSim.java
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepCallbackHandler.java java_src/dk/robwork/ThreadSimulatorStepCallbackHandler.java
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEvent.java java_src/dk/robwork/ThreadSimulatorStepEvent.java
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEventListener.java java_src/dk/robwork/ThreadSimulatorStepEventListener.java
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEventDispatcher.java java_src/dk/robwork/ThreadSimulatorStepEventDispatcher.java
        		COMMAND cmake -E echo "Compiling Java files..."
                COMMAND cmake -E make_directory java_build/dk/robwork
                COMMAND ${Java_JAVAC_EXECUTABLE} -cp ${RW_LIBS}/rw_java.jar -d ${CMAKE_CURRENT_BINARY_DIR}/java_build java_src/dk/robwork/*.java
                COMMAND cmake -E echo "Creating jar file..."
                COMMAND ${Java_JAR_EXECUTABLE} cvf ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/rwsim_java.jar  -C java_build .
    			COMMAND cmake -E echo "Creating Javadoc..."
            	COMMAND cmake -E make_directory ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
            	COMMAND ${Java_JAVADOC_EXECUTABLE}
            					-classpath ${RW_LIBS}/rw_java.jar
            					-d ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
            					-windowtitle "RobWorkSim Java API Documentation"
            					-public
            					-sourcepath java_src dk.robwork
            					-linkoffline ${RW_LIBS}/javadoc ${RW_LIBS}/javadoc
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            )
            SET(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
        ELSE()
            MESSAGE(STATUS "RobWorkSim: Java bindings DISABLED!")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "RobWorkSim: Java bindings DISABLED")
    ENDIF()		
	
ENDIF()
