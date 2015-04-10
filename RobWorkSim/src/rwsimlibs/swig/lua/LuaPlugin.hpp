/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef RWSIM_SWIG_LUAPLUGIN_HPP_
#define RWSIM_SWIG_LUAPLUGIN_HPP_

#include <rw/common/Plugin.hpp>

#ifdef __cplusplus
extern "C" {
#endif

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#ifdef __cplusplus
}
#endif

namespace rwsim {
namespace swig {

    /**
     * @brief initialize a lua state
     * @param L
     * @return
     */
    int openLuaLibRWSim(lua_State* L);

    /**
     * @brief A Lua plugin that define extensions for rwlibs.swig.LuaState.LuaLibrary
     */
    class LuaPlugin: public rw::common::Plugin {
    public:

        /**
         * @brief constructor
         */
        LuaPlugin();

        //! destructor
        virtual ~LuaPlugin();

        //! @copydoc rw::common::Plugin::getExtensionDescriptors
        std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

        //! @copydoc rw::common::Plugin::makeExtension
        rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);

    };

}
}
#endif /* LUA_HPP_ */
