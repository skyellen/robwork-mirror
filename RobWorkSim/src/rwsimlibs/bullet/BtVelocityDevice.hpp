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

#ifndef RWSIMLIBS_BULLET_BTVELOCITYDEVICE_HPP_
#define RWSIMLIBS_BULLET_BTVELOCITYDEVICE_HPP_

/**
 * @file BtVelocityDevice.hpp
 *
 * \copydoc rwsimlibs::bullet::BtVelocityDevice
 */

#include "BtDevice.hpp"

#include <rw/common/Ptr.hpp>

#include <vector>

namespace rwsim { namespace dynamics { class RigidDevice; } }

class btTypedConstraint;

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief A velocity device.
 */
class BtVelocityDevice: public BtDevice {
public:
	//! @brief Constructor.
	BtVelocityDevice(rw::common::Ptr<rwsim::dynamics::RigidDevice> rdev, const std::vector<btTypedConstraint*>& constraints);

	//! @brief Destructor.
	virtual ~BtVelocityDevice();

	//! @brief @copydoc BtDevice::udpate
	virtual void update(double dt, rw::kinematics::State& state);

	//! @brief @copydoc BtDevice::postUpdate
	virtual void postUpdate(rw::kinematics::State& state);

private:
	const rw::common::Ptr<rwsim::dynamics::RigidDevice> _rdev;
    const std::vector<btTypedConstraint*> _constraints;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTVELOCITYDEVICE_HPP_ */
