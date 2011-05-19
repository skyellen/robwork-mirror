/*
 * LuaRobWork.cpp
 *
 *  Created on: 02/03/2010
 *      Author: jimali
 */
#include "LuaRobWork.hpp"

#include "LuaMathStub.hpp"
#include "LuaKinematicsStub.hpp"
#include "LuaModelsStub.hpp"
#include "LuaSensorStub.hpp"
#include "LuaLoadersStub.hpp"
#include "LuaTrajectoryStub.hpp"
#include "LuaProximityStub.hpp"
#include "LuaCommonStub.hpp"
#include "LuaPathPlanningStub.hpp"


int rwlua::rw::luaRobWork_open (lua_State* tolua_S){
	tolua_LuaMath_open(tolua_S);
	tolua_LuaKinematics_open(tolua_S);
	tolua_LuaLoaders_open(tolua_S);
	tolua_LuaModels_open(tolua_S);
	tolua_LuaSensor_open(tolua_S);

	tolua_LuaTrajectory_open(tolua_S);
	tolua_LuaCommon_open(tolua_S);
	tolua_LuaPathPlanning_open(tolua_S);
	return tolua_LuaProximity_open(tolua_S);
}
