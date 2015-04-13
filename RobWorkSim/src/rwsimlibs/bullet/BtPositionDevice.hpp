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

#ifndef RWSIMLIBS_BULLET_BTPOSITIONDEVICE_HPP_
#define RWSIMLIBS_BULLET_BTPOSITIONDEVICE_HPP_

/**
 * @file BtPositionDevice.hpp
 *
 * \copydoc rwsimlibs::bullet::BtPositionDevice
 */

#include "BtDevice.hpp"

#include <rw/common/Ptr.hpp>

#include <vector>

namespace rw { namespace kinematics { class Frame; } }
namespace rwsim { namespace dynamics { class KinematicDevice; } }

class btRigidBody;

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief A position device.
 */
class BtPositionDevice: public BtDevice {
public:
	//! @brief Definition of a RobWork frame and Bullet body pair.
	typedef std::pair<const rw::kinematics::Frame*, btRigidBody*> FrameBodyPair;

	/**
	 * @brief Constructor.
	 * @param dev [in] a kinematic device.
	 * @param frameToBtBody [in] a list of pairs of Frames and Bullet bodies.
	 */
	BtPositionDevice(rw::common::Ptr<rwsim::dynamics::KinematicDevice> dev, const std::vector<FrameBodyPair>& frameToBtBody);

	//! @brief Destructor.
	virtual ~BtPositionDevice();

	//! @brief @copydoc BtDevice::udpate
	virtual void update(double dt, rw::kinematics::State& state);

	//! @brief @copydoc BtDevice::postUpdate
	virtual void postUpdate(rw::kinematics::State& state);

private:
    const rw::common::Ptr<rwsim::dynamics::KinematicDevice> _kdev;
    const std::vector<FrameBodyPair> _frameToBtBody;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTPOSITIONDEVICE_HPP_ */
