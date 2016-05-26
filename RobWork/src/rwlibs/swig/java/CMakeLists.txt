if(POLICY CMP0046)
  cmake_policy(SET CMP0046 OLD)
endif()

SET(SUBSYS_NAME rw_java )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from java." )
SET(SUBSYS_DEPS rw )

SET(build TRUE)

FIND_PACKAGE(SWIG 1.3 QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

    FIND_PACKAGE(Java)	
    FIND_PACKAGE(JNI)
    if( NOT (JAVA_FOUND AND JNI_FOUND) )
        set(DEFAULT false)
        set(REASON "JAVA or JNI not found!")
    endif()
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    INCLUDE(UseJava)
    INCLUDE(UseSWIG)
    
    SET_SOURCE_FILES_PROPERTIES(../rw.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rw.i PROPERTIES SWIG_FLAGS "-includeall")
    
   INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
    set(CMAKE_SWIG_FLAGS "-package" "dk.robwork")
    # Put java files in different directory suitable for JAR generation later on
    SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/dk/robwork)
    # SWIG
    SWIG_ADD_MODULE(rw_jni java ../rw.i)
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
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../rw.i"
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
    
ENDIF()
