SET(SUBSYS_NAME rwsim_python )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from python." )
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

    FIND_PACKAGE(PythonInterp QUIET)
    FIND_PACKAGE(PythonLibs QUIET)

    if( NOT (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND) )
        set(DEFAULT false)
        set(REASON "PYTHONLIBS AND PYTHONINTERP not found!")
    endif()
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
INCLUDE(UseSWIG)
    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
    SET_SOURCE_FILES_PROPERTIES(../rwsim.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rwsim.i PROPERTIES SWIG_FLAGS "-includeall")
	
    SET(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    SWIG_ADD_MODULE(rwsim python ../rwsim.i ../ScriptTypes.cpp)
    SWIG_LINK_LIBRARIES(rwsim rwsim
        ${ROBWORK_LIBRARIES} 
        ${PYTHON_LIBRARIES} )
     
ENDIF()