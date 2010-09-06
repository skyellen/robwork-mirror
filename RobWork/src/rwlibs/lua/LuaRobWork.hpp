/*
 * LuaRobWork.hpp
 *
 *  Created on: 02/03/2010
 *      Author: jimali
 */

#ifndef LUAROBWORK_HPP_
#define LUAROBWORK_HPP_

#include "LuaMath.hpp"
#include "LuaKinematics.hpp"
#include "LuaModels.hpp"
#include "LuaSensor.hpp"
#include "LuaLoaders.hpp"
#include "LuaTrajectory.hpp"
#include "LuaProximity.hpp"
#include "LuaCommon.hpp"

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "tolua++.h"
}
namespace rwlibs {
namespace lua {
    //! @addtogroup lua
    // @{

    /**
     * @brief initializes a lua state with the robwork functionality
     * @param tolua_S [in] lua state
     * @return
     */
	int  luaRobWork_open (lua_State* tolua_S);

	// @}
}
}
#endif /* LUAROBWORK_HPP_ */
