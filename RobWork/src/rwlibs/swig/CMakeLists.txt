SET(SUBSYS_NAME rw_swig)
set(SUBSYS_DESC "Swig interfaces for RobWork")
SET(SUBSYS_DEPS rw rw_simulation rw_pathplanners rw_pathoptimization rw_task)

if( SWIG_FOUND )
    set(DEFAULT TRUE)
    set(REASON)
else()
    set(DEFAULT FALSE)
    set(REASON "SWIG was not found.")
endif()

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF( build ) 
	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(rw.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(rw.i PROPERTIES SWIG_FLAGS "-includeall")

	
	# lua interface generation
	STRING(REPLACE "rw_lua" "" LIBS_WITHOUT_RW_LUA ${ROBWORK_LIBRARIES})
    SWIG_ADD_MODULE(rw_lua lua rw.i ScriptTypes.cpp)
	SWIG_LINK_LIBRARIES(rw_lua rw_algorithms
                              rw_pathplanners
                              rw_pathoptimization
                              rw_simulation
                              rw_opengl
                              rw_assembly
                              ${LUA_LIBRARIES}
                              rw_proximitystrategies
                              rw_task
                              ${YAOBI_LIBRARIES}
                              ${PQP_LIBRARIES}  
                              rw
                              ${OPENGL_LIBRARIES}
                              ${XERCESC_LIBRARIES}
                              ${ASSIMP_LIBRARIES}
                              ${ZLIB_LIBRARIES}
                              ${MINIZIP_LIBRARIES}
                              ${Boost_LIBRARIES}
                              ${LAPACK_LIBRARIES} 
                              ${BLAS_LIBRARIES}
                              rw_qhull)
    ADD_DEPENDENCIES(rw_lua rw_algorithms
                          rw_pathplanners
                          rw_pathoptimization
                          rw_simulation
                          rw_opengl
                          rw_proximitystrategies
                          rw_task
                          ${YAOBI_LIBRARIES}
                          ${PQP_LIBRARIES}  
                          rw
                          rw_qhull
                          ${ASSIMP_LIBRARIES}
                          ${ZLIB_LIBRARIES}
                          ${MINIZIP_LIBRARIES}
)
    
    ADD_LIBRARY(rw_lua_s STATIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
    LINK_LIBRARIES(rw_lua_s rw)

	# the dynamic version
    #ADD_LIBRARY(rw_lua DYNAMIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
    #LINK_LIBRARIES(rw_lua rw)
    
	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(rw_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(rw_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()

    install(TARGETS rw_lua rw_lua_s 
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT swig
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig)
    

	FIND_PACKAGE(PythonInterp)
    FIND_PACKAGE(PythonLibs)
    INCLUDE(CMakeDependentOption)
    #CMAKE_DEPENDENT_OPTION(RW_PYTHON_BINDINGS_ENABLED "Enable python bindings." 
    #                   ON "PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND" OFF)
    IF( RW_PYTHON_BINDINGS_ENABLED )

        IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
            MESSAGE(STATUS "RobWork: Python bindings ENABLED!")
            INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
            SWIG_ADD_MODULE(rw python rw.i)
	        SWIG_LINK_LIBRARIES(rw ${ROBWORK_LIBRARIES} ${PYTHON_LIBRARIES} )
	        #ADD_DEPENDENCIES(rw rw_algorithms
            #              rw_pathplanners
            #              rw_pathoptimization
            #              rw_simulation
            #              rw_opengl
            #              rw_proximitystrategies
            #              ${YAOBI_LIBRARIES}
            #              ${PQP_LIBRARIES}  
            #              rw
            #              rw_qhull)
	        # TODO: 
	        #install(TARGETS rw 
            #    RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT swig
            #    LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
            #    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig)
	        
        ELSE()
            MESSAGE(STATUS "RobWork: Python bindings DISABLED!")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "RobWork: Python bindings DISABLED")
    ENDIF() 	
	
	
	FIND_PACKAGE(Java)	
    FIND_PACKAGE(JNI)

    INCLUDE(CMakeDependentOption)
    CMAKE_DEPENDENT_OPTION(RW_JAVA_BINDINGS_ENABLED "Enable java bindings." 
                       ON "JAVA_FOUND AND JNI_FOUND" OFF)
    IF( RW_JAVA_BINDINGS_ENABLED )

        IF( JAVA_FOUND AND JNI_FOUND )
        	INCLUDE(UseJava)
        
            MESSAGE(STATUS "RobWork: Java bindings ENABLED!")
            INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
            set(CMAKE_SWIG_FLAGS "-package" "dk.robwork")
            # Put java files in different directory suitable for JAR generation later on
            SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/dk/robwork)
            # SWIG
            SWIG_ADD_MODULE(rw_jni java rw.i)
            SWIG_LINK_LIBRARIES(rw_jni ${ROBWORK_LIBRARIES} )
	        ADD_DEPENDENCIES(rw_jni rw_algorithms
                          rw_pathplanners
                          rw_pathoptimization
                          rw_simulation
                          rw_opengl
                          rw_proximitystrategies
                          rw_task
                          ${YAOBI_LIBRARIES}
                          ${PQP_LIBRARIES}  
                          rw
                          rw_qhull)
            # Force removal of previous Java compilation and source when interface file changes
            # This is required as types may be removed or change name
            # (in this case previous java classes would interfere with current compilation).
			ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
                COMMAND cmake -E remove_directory java_src
                COMMAND cmake -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
                DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/rw.i"
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Removing old Java source..."
            )
            ADD_CUSTOM_TARGET(CleanDep DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep )
            ADD_DEPENDENCIES(rw_jni CleanDep)
            # Compile java code and create JAR and Javadoc
			ADD_CUSTOM_COMMAND(TARGET rw_jni
				POST_BUILD
        		COMMAND cmake -E echo "Removing old Java compilation..."
                COMMAND cmake -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build"
            	COMMAND cmake -E remove_directory "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc"
        		COMMAND cmake -E echo "Copying Java source..."
                COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRW.java java_src/dk/robwork/LoaderRW.java
        		COMMAND cmake -E echo "Compiling Java files..."
                COMMAND cmake -E make_directory java_build/dk/robwork
                COMMAND ${Java_JAVAC_EXECUTABLE} -d ${CMAKE_CURRENT_BINARY_DIR}/java_build java_src/dk/robwork/*.java
                COMMAND cmake -E echo "Creating jar file..."
                COMMAND ${Java_JAR_EXECUTABLE} cvf ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/rw_java.jar  -C java_build .
    			COMMAND cmake -E echo "Creating Javadoc..."
            	COMMAND cmake -E make_directory ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
            	COMMAND ${Java_JAVADOC_EXECUTABLE} -d ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc -windowtitle "RobWork Java API Documentation" -public -sourcepath java_src dk.robwork
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            )
            SET(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
        ELSE()
            MESSAGE(STATUS "RobWork: Java bindings DISABLED!")
        ENDIF()
    ELSE()
        MESSAGE(STATUS "RobWork: Java bindings DISABLED")
    ENDIF() 	

	
	RW_ADD_INCLUDES(swig "rwlibs/swig" Lua.hpp ScriptTypes.hpp)
	
	
ENDIF()






