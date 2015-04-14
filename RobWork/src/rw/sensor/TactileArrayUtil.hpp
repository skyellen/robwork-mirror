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


#ifndef TACTILEARRAYUTIL_HPP_
#define TACTILEARRAYUTIL_HPP_

#include <vector>

#include <rw/kinematics/State.hpp>
#include <rw/sensor/Contact3D.hpp>
#include <rw/sensor/TactileArrayModel.hpp>

namespace rw {
namespace sensor {


/**
 * @brief Utillity class for general computations on a tactile array
 */
class TactileArrayUtil {

public:

	/**
	 * @brief Estimate the contacts on the tactile array sensor.
	 * @param arraySensor [in] the array sensor that describe the tactile array
	 * @param state [in] the current state of the system
	 * @param minContactForce [in] A threshold value that determines when a force is a contact force
	 * and not just noise.
	 * @return All estimated contacts
	 */
    static std::vector<Contact3D>
        estimateContacts(const rw::sensor::TactileArrayModel& arraySensor,
                         const rw::kinematics::State& state,
                         double minContactForce);

};

}
}
#endif /* TACTILEARRAYUTIL_HPP_ */
