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


#ifndef RWSIM_SIMULATOR_ODEMATERIALMAP_HPP_
#define RWSIM_SIMULATOR_ODEMATERIALMAP_HPP_

#include <ode/ode.h>
#include <vector>

namespace rwsim { namespace dynamics { class MaterialDataMap; } }
namespace rwsim { namespace dynamics { class ContactDataMap; } }

namespace rwsim {
namespace simulator {
	class ODEBody;

	//! @brief The ODE material map is responsible for the modelling of contact dynamics, such as friction and restitution phenomena.
	class ODEMaterialMap {
	public:
		/**
		 * @brief Constructor
		 * @param map
		 * @param cmap
		 * @param odeBodies
		 * @return
		 */
		ODEMaterialMap(dynamics::MaterialDataMap& map,
					   dynamics::ContactDataMap& cmap,
					   std::vector<ODEBody*> odeBodies);

		/**
		 * @brief copies contact properties between body \b b1 and body \b b2 into
		 * the dContact.
		 * @param con
		 * @param b1
		 * @param b2
		 */
		void setContactProperties(dContact &con, ODEBody *b1, ODEBody *b2);

	private:
		dynamics::MaterialDataMap &_map;
		dynamics::ContactDataMap &_cmap;

		std::vector<float> _muMap;
		std::vector<float> _bounceMap;
		std::vector<float> _bounceVelMap;
		std::vector<float> _cfmMap;
		std::vector<float> _erpMap;
	};
}
}

#endif /* ODEMATERIALMAP_HPP_ */
