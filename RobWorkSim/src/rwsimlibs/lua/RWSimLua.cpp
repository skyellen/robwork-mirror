/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "RWSimLua.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>

extern "C" {
#include "tolua++.h"
}

#include "RWSimLuaStub.hpp"

#include <iostream>

using namespace rw::kinematics;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rwlibs::lua;

namespace
{
    // Pushes the RobWork module table on the stack.
    void enterRobWorkModule(lua_State* L)
    {
        lua_getfield(L, LUA_GLOBALSINDEX, "rwlibs");
        lua_getfield(L, -1, "lua");
        lua_getfield(L, -1, "internal");
    }

    // Pops module tables from the stack.
    void leaveRobWorkModule(lua_State* L)
    {
        lua_pop(L, 3);
    }

    // Save a userdata pointer under the name 'name' in the RobWork package.
    void setPtr(lua_State* L, const std::string& name, void* data)
    {
        enterRobWorkModule(L);

        void** ptr = (void**)lua_newuserdata(L, sizeof(data));
        *ptr = data;
        lua_setfield(L, -2, name.c_str());

        leaveRobWorkModule(L);
    }

    // A global variable to handle changes to the common Lua state.
    void ignoreState(const State& state) {}
    RobWork::StateChangedListener stateChangedListener(ignoreState);

    void ignorePath(const TimedStatePath& path) {}
    RobWork::PathChangedListener pathChangedListener(ignorePath);
}

int RobWork::open(lua_State* L)
{
    int error = tolua_robwork_wrapper_open(L);
    return error;
}

void RobWork::setOutput(lua_State* L, Output* output)
{
    setPtr(L, "output", output);
}

void RobWork::setState(
    lua_State* L,
    State* state)
{
    setPtr(L, "state", state);
}

void RobWork::setDynamicWorkCell(lua_State* L, dynamics::DynamicWorkCell* workcell)
{
    setPtr(L, "workcell", workcell);
}

void RobWork::setCollisionDetector(
    lua_State* L,
    rw::proximity::CollisionDetector* strategy)
{
    setPtr(L, "collisionDetector", strategy);
}

void RobWork::setCollisionStrategy(
    lua_State* L,
    rw::proximity::CollisionStrategy* strategy)
{
    setPtr(L, "collisionStrategy", strategy);
}

void RobWork::setStateChangedListener(
    const StateChangedListener& listener)
{
    stateChangedListener = listener;
}

const RobWork::StateChangedListener& RobWork::getStateChangedListener()
{
    return stateChangedListener;
}

void RobWork::setPathChangedListener(
    const PathChangedListener& listener)
{
    pathChangedListener = listener;
}

const RobWork::PathChangedListener& RobWork::getPathChangedListener()
{
    return pathChangedListener;
}
