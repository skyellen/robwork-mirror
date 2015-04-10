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

#ifndef RWLIBS_SWIG_LUASTATE_HPP_
#define RWLIBS_SWIG_LUASTATE_HPP_

#include <rw/common/ExtensionPoint.hpp>
#include <vector>

// forward declaration
struct lua_State;

namespace rwlibs {
namespace swig {


/**
 * @brief a robwork wrapper for the lua_State struct. The standard robwork lua libs
 * will be initialized automatically. Also this provides an extension point for
 * adding user defined lua enabled libraries.
 */
class LuaState {
public:
	//! smart pointer type of LuaState
	typedef rw::common::Ptr<LuaState> Ptr;

    //! @brief constructor
    LuaState();

    //! @brief destructor
    virtual ~LuaState();

    //! @brief reset this luastate
    void reset();

    //! @brief run a lua command block
    int runCmd(const std::string& str);


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

    //! remove specific library from luastate
    void removeLibrary(const std::string& id);

    //! @brief get the lua_State
    lua_State* get() { return _lua;}

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwlibs::swig::LuaState::Factory,rwlibs::swig::LuaState::LuaLibrary,rwlibs.swig.LuaState.LuaLibrary}
	 */

	/**
	 * @brief a factory for LuaLibrary. This factory also defines an
	 * extension point for LuaLibraries. This permit users to define extensions to
	 * the lua interfaces through RobWork extension.
	 */
    class Factory: public rw::common::ExtensionPoint<LuaLibrary> {
    public:
    	//! constructor
        Factory():rw::common::ExtensionPoint<LuaLibrary>("rwlibs.swig.LuaState.LuaLibrary", "Extension point for Lua add-on libraries, acked in robwork plugins."){};

        /**
         * @brief get a specific lua library based on id
         * @param id [in] string identifier of lua library
         * @return a lualibrary if matching lib exists else NULL
         */
        static rw::common::Ptr<LuaLibrary> getLuaLibrary(const std::string& id);

        /**
         * @brief get all avaliable lua libraries
         * @return all avalilable lua libraries
         */
        static std::vector<LuaLibrary::Ptr> getLuaLibraries();

        /**
         * @brief get a list of supported lua libraries
         * @return
         */
        static std::vector<std::string> getLuaLibraryIDs();

    };

private:
    struct lua_State *_lua;
    std::vector< LuaLibrary::Ptr > _libraryCBs;
};

}
}

#endif /* SHAREDLUASTATE_HPP_ */
