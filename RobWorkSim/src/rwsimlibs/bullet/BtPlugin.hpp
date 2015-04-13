/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_BULLET_BTPLUGIN_HPP_
#define RWSIMLIBS_BULLET_BTPLUGIN_HPP_

/**
 * @file BtPlugin.hpp
 *
 * \copydoc rwsimlibs::bullet::BtPlugin
 */

#include <rw/common/Plugin.hpp>

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief A Bullet plugin that provides additional functionality to the
 * rwsim::simulator::PhysicsEngine::Factory through the plugin structure.
 *
 * The following extension is added with the use of this plugin:
 *  - Bullet
 */
class BtPlugin: public rw::common::Plugin {
public:
	//! @brief Construct new plugin
	BtPlugin();

	//! @brief Destructor
	virtual ~BtPlugin();

    //! @copydoc rw::common::Plugin::getExtensionDescriptors
	std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

    //! @copydoc rw::common::Plugin::makeExtension
	rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTPLUGIN_HPP_ */
