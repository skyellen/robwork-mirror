#FIND_PACKAGE(Tolua++)

#FIND_PACKAGE(SWIG 1.3)
#MESSAGE(STATUS "${SWIG_FOUND}")
#MESSAGE(STATUS "${SWIG_DIR}")
#MESSAGE(STATUS "${SWIG_EXECUTABLE}")
#MESSAGE(STATUS "${SWIG_VERSION}")

IF( SWIG_FOUND ) 

	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(rw.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(rw.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
    SWIG_ADD_MODULE(rwlua lua rw.i ScriptTypes.cpp)
	SWIG_LINK_LIBRARIES(rwlua rw ${ROBWORK_LIBRARIES})

    ADD_LIBRARY(rw_lua STATIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
    LINK_LIBRARIES(rw_lua rw ${ROBWORK_LIBRARIES})
	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(rw_lua PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(rw_lua PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()
	
	# PYthon interface generation
	FIND_PACKAGE(PythonInterp)
	FIND_PACKAGE(PythonLibs)
	#MESSAGE("PYTHONINTERP_FOUND:  ${PYTHONINTERP_FOUND}" )
	#MESSAGE("PYTHONLIBS_FOUND:  ${PYTHONLIBS_FOUND}" )
	IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
	    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
	    SWIG_ADD_MODULE(rw python rw.i)
	    SWIG_LINK_LIBRARIES(rw ${ROBWORK_LIBRARIES} ${PYTHON_LIBRARIES} )
	    
	    # TODO we need to invoke python distutils to compile a shared library that has the c++ objects available.
	    
	ENDIF()  
	
	
ENDIF()
