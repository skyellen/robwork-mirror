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
 
#ifndef _SIMPLEFINGER_PLUGIN_HPP
#define _SIMPLEFINGER_PLUGIN_HPP

#include <rw/common/Plugin.hpp>

namespace rwlibs {
namespace geometry {
namespace simplefinger {

/**
 * A plugin that adds a simple gripper finger shape as a custom
 * geometry type to be loaded from the workcell.
 */
class SimpleFingerPlugin: public rw::common::Plugin {
public:

	SimpleFingerPlugin();
	
	~SimpleFingerPlugin();
	
	//! @copydoc Plugin::getExtensionDescriptors
	std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();
	
	//! @copydoc Plugin::makeExtension
	rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);
};

} /* simplefinger */
} /* geometry */
} /* rwlibs */

#endif
