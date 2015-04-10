/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef RWSIM_SWIG_LUAPLUGIN_HPP_
#define RWSIM_SWIG_LUAPLUGIN_HPP_

#include <rw/common/Plugin.hpp>
#include "Lua.hpp"
namespace rwslibs {
namespace swig {

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
