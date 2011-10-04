#FIND_PACKAGE(Tolua++)

FIND_PACKAGE(SWIG 1.3)
MESSAGE(STATUS "${SWIG_FOUND}")
MESSAGE(STATUS "${SWIG_DIR}")
MESSAGE(STATUS "${SWIG_EXECUTABLE}")
MESSAGE(STATUS "${SWIG_VERSION}")

IF( SWIG_FOUND ) 
	MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(rw.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(rw.i PROPERTIES SWIG_FLAGS "-includeall")

    SWIG_ADD_MODULE(rwlua lua rw.i)
    SWIG_LINK_LIBRARIES(rwlua rw)
	
	FIND_PACKAGE(PythonInterp)
	FIND_PACKAGE(PythonLibs)
	MESSAGE("PYTHONINTERP_FOUND:  ${PYTHONINTERP_FOUND}" )
	MESSAGE("PYTHONLIBS_FOUND:  ${PYTHONLIBS_FOUND}" )
	IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
	    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
	    SWIG_ADD_MODULE(rwl python rw.i)
	    SWIG_LINK_LIBRARIES(rwl ${ROBWORK_LIBRARIES} ${PYTHON_LIBRARIES} )
	ENDIF()  
	
	
ENDIF()



SET(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# if we don't find lua and use lua is enabled then we assume that it will 
# be compiled with RobWork
IF( USE_LUA )
    SET(SRC_FILES LuaRobWork.cpp)
    SET(SRC_FILES_HPP LuaRobWork.hpp)
    
    ADD_TOLUA_PACKAGE("LuaMath" "LuaMathStub")
    ADD_TOLUA_PACKAGE("LuaKinematics" "LuaKinematicsStub")
    ADD_TOLUA_PACKAGE("LuaLoaders" "LuaLoadersStub")
    ADD_TOLUA_PACKAGE("LuaSensor" "LuaLoadersStub")
    ADD_TOLUA_PACKAGE("LuaModels" "LuaModelsStub")
    ADD_TOLUA_PACKAGE("LuaTrajectory" "LuaTrajectoryStub") 
    ADD_TOLUA_PACKAGE("LuaProximity" "LuaProximityStub")
    ADD_TOLUA_PACKAGE("LuaCommon" "LuaCommonStub")
    ADD_TOLUA_PACKAGE("LuaPathPlanning" "LuaPathPlanningStub")
	ADD_TOLUA_PACKAGE("LuaGraphics" "LuaGraphicsStub")
	
    #MESSAGE( "SRC_FILES ${SRC_FILES}.... HEADER FILES: ${SRC_FILES_HPP}")   
    ADD_LIBRARY(rw_lua ${SRC_FILES} ${SRC_FILES_HPP})
    ADD_DEPENDENCIES(rw_lua tolua)
    
    INSTALL(TARGETS rw_lua DESTINATION ${RW_LIB_INSTALL_DIR})
    #INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/lua)
ENDIF()
