#FIND_PACKAGE(Tolua++)

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
