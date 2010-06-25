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

#ifndef RWSIM_DYNAMICS_DYNAMICWORKCELLLOADER_HPP_
#define RWSIM_DYNAMICS_DYNAMICWORKCELLLOADER_HPP_

#include <rwsim/dynamics/DynamicWorkcell.hpp>

namespace rwsim {
namespace loaders {

	/**
	 * @brief class for loading of dynamic owrkcells
	 */
    class DynamicWorkCellLoader
    {
    public:
    	/**
    	 * @brief load a workcell
    	 * @param filename
    	 */
        static rw::common::Ptr<dynamics::DynamicWorkcell>
        	load(const std::string& filename);

    };

}
}

#endif /*DYNAMICWORKCELLLOADER_HPP_*/
