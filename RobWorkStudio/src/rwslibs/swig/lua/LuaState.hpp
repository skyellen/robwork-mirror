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

#ifndef RWS_LUASTATE_HPP_
#define RWS_LUASTATE_HPP_

#include <sstream>
#include <rw/common/StringUtil.hpp>
#include <rws/RobWorkStudio.hpp>

struct lua_State;


namespace rws {

/**
 * @brief a robworkstudio wrapper for the lua_State struct. The standard robwork lua libs
 * will be initialized automatically.
 */
class LuaState {
public:
	typedef rw::common::Ptr<LuaState> Ptr;

    //! @brief constructor
    LuaState();

    //! @brief destructor
    virtual ~LuaState();

    //! @brief reset this luastate
    void reset();

    //! @brief run a lua command block
    int runCmd(const std::string& str);

    /**
     * @brief
     * @param rws
     */
    void setRobWorkStudio(rws::RobWorkStudio* rws){
        _rws = rws;
    }

    //! type of the addlibrary callback function
    //typedef boost::function<int(lua_State*)> AddLibraryCB;

    struct LuaLibrary {
    	typedef rw::common::Ptr<LuaLibrary> Ptr;
    	virtual ~LuaLibrary() {};
    	virtual const std::string getId() = 0;
    	virtual bool initLibrary(LuaState& state) = 0;
    };

    /**
     * @brief when the LuaState is reset all library constributers will be asked
     * to add their libraries to the state again.
     */
    void addLibrary(LuaLibrary::Ptr lib);
    void removeLibrary(const std::string& id);

    //! @brief get the lua_State
    lua_State* get() { return _lua;}

private:
    struct lua_State *_lua;
    rws::RobWorkStudio* _rws;
    std::vector< LuaLibrary::Ptr > _libraryCBs;
};

}

#endif /* SHAREDLUASTATE_HPP_ */
