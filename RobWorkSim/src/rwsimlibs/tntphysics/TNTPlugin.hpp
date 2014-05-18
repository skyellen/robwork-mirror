/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTPLUGIN_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTPLUGIN_HPP_

/**
 * @file TNTPlugin.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTPlugin
 */

#include <rw/common/Plugin.hpp>

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A loader plugin for TNT physics engines. It provides
 * additional functionality to the rwsim::simulator::PhysicsEngine::Factory
 * through the plugin structure.
 *
 * Three extensions are added with the use of this plugin, namely
 *  - TNTPhysics
 *  - TNTWorld
 *  - TNTIsland
 */
class TNTPlugin: public rw::common::Plugin {
public:
	//! @brief Construct new plugin
	TNTPlugin();

	//! @brief Destructor
	virtual ~TNTPlugin();

	//! @copydoc rw::common::Plugin::getExtensionDescriptors
    std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

	//! @copydoc rw::common::Plugin::makeExtension
    rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& id);

    //! @brief Register the plugins extensions in the rw::common::ExtensionRegistry.
    static void registerPlugin();
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTPLUGIN_HPP_ */
