/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "RobWork.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>

extern "C" {
#include "tolua++.h"
}

#include "robwork_stub.hpp"

#include <iostream>

using namespace rw::kinematics;
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

void RobWork::setState(lua_State* L, State* state)
{
    setPtr(L, "state", state);
}

void RobWork::setWorkCell(lua_State* L, WorkCell* workcell)
{
    setPtr(L, "workcell", workcell);
}

void RobWork::setPathPlannerFactory(lua_State* L, PathPlannerFactory* factory)
{
    setPtr(L, "pathPlannerFactory", factory);
}
