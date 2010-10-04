/*
 * SharedLuaState.hpp
 *
 *  Created on: 04/10/2010
 *      Author: jimali
 */

#ifndef LUASTATE_HPP_
#define LUASTATE_HPP_



#include <sstream>
#include <rw/common/StringUtil.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/lua/LuaRobWork.hpp>
#include <rws/RobWorkStudio.hpp>

struct lua_State;

class LuaState {
public:
    LuaState():_rws(NULL){}

    void reset();

    void setRobWorkStudio(rws::RobWorkStudio* rws){
        std::cout << "Set robworkStudio: " << rws << std::endl;
        _rws = rws;
    }

    lua_State* get() { return _lua.get();}

private:
    rw::common::Ptr<struct lua_State> _lua;
    rws::RobWorkStudio* _rws;
};

#endif /* SHAREDLUASTATE_HPP_ */
