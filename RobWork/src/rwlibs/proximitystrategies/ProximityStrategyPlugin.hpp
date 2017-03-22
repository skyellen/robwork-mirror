/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPLUGIN_HPP_
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPLUGIN_HPP_

/**
 * @file ProximityStrategyPlugin.hpp
 *
 * \copydoc rwlibs::proximitystrategies::ProximityStrategyPlugin
 */

#include <rw/common/Plugin.hpp>

namespace rwlibs {
namespace proximitystrategies {
//! @addtogroup proximitystrategies

//! @{
/**
 * @brief A plugin providing proximity strategies for RobWork.
 *
 * The following extensions are added with the use of this plugin (if RobWork is configured to use them):
 *  - Bullet - rwlibs::proximitystrategies::ProximityStrategyBullet - Bullet Physics
 *  - FCL - rwlibs::proximitystrategies::ProximityStrategyFCL - Flexible Collision Library
 *  - PQP - rwlibs::proximitystrategies::ProximityStrategyPQP - Proximity Query Package
 *  - Yaobi - rwlibs::proximitystrategies::ProximityStrategyYaobi - Yaobi
 */
class ProximityStrategyPlugin: public rw::common::Plugin {
public:
	//! @brief Construct new plugin
	ProximityStrategyPlugin();

	//! @brief Destructor
	virtual ~ProximityStrategyPlugin();

	//! @copydoc rw::common::Plugin::getExtensionDescriptors
    std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

	//! @copydoc rw::common::Plugin::makeExtension
    rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& id);

    //! @brief Register the plugins extensions in the rw::common::ExtensionRegistry.
    static void registerPlugin();
};
//! @}
} /* namespace proximitystrategies */
} /* namespace rwlibs */

#endif /* RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPLUGIN_HPP_ */
