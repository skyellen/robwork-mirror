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

#ifndef RWSIMLIBS_BULLET_BTDEVICE_HPP_
#define RWSIMLIBS_BULLET_BTDEVICE_HPP_

/**
 * @file BtDevice.hpp
 *
 * \copydoc rwsimlibs::bullet::BtDevice
 */

namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief An interface for Bullet devices.
 */
class BtDevice {
public:
	//! @brief Destructor.
	virtual ~BtDevice() {};

	/**
	 * @brief Update device.
	 * @param dt [in] the timestep.
	 * @param state [in/out] the state to update.
	 */
	virtual void update(double dt, rw::kinematics::State& state) = 0;

	/**
	 * @brief Post update of device.
	 * @param state [in/out] the result of the update.
	 */
	virtual void postUpdate(rw::kinematics::State& state) = 0;

protected:
	BtDevice() {};
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTDEVICE_HPP_ */
