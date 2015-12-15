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

#ifndef RWSIMLIBS_RWPE_RWPEBODYKINEMATIC_HPP_
#define RWSIMLIBS_RWPE_RWPEBODYKINEMATIC_HPP_

/**
 * @file RWPEBodyKinematic.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEBodyKinematic
 */

#include <rw/common/Ptr.hpp>
#include "RWPEBody.hpp"

// Forward declarations
namespace rwsim { namespace dynamics { class KinematicBody; } }

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The RWPEBodyKinematic is a wrapper for a rwsim::dynamics::KinematicBody and is used for bodies
 * that moves with a dictated velocity.
 *
 * The motion of a kinematic body can not be influenced by forces. The kinematic body can however impose
 * forces on other objects that it is in contact with or is constrained in other ways.
 */
class RWPEBodyKinematic: public RWPEBody {
public:
	/**
	 * @brief Construct a new kinematic body.
	 * @param body [in] a pointer to the underlying rwsim::dynamics::KinematicBody.
	 */
	RWPEBodyKinematic(rw::common::Ptr<rwsim::dynamics::KinematicBody> body);

	//! @brief Destructor.
	virtual ~RWPEBodyKinematic();

	/**
	 * @brief Get the wrapped rwsim::dynamics::KinematicBody
	 * @return the native object.
	 */
	rw::common::Ptr<rwsim::dynamics::KinematicBody> getKinematicBody() const;

	/**
	 * @brief Integrate the motion.
	 * @param stepsize [in] the size of the step to integrate.
	 * @param islandState [in/out] the state with the current configuration (position) of the body, state is updated with new position.
	 * @param rwstate [in/out] the rw::kinematics::State which dictates the velocities.
	 */
	virtual void integrate(double stepsize, RWPEIslandState &islandState, const rw::kinematics::State &rwstate) const;

	//! @copydoc RWPEBody::updateRW
	virtual void updateRW(rw::kinematics::State &rwstate, const RWPEIslandState &islandState) const;

	//! @copydoc RWPEBody::getVelocityW
	virtual rw::math::VelocityScrew6D<> getVelocityW(const rw::kinematics::State &rwstate, const RWPEIslandState &islandState) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEBODYKINEMATIC_HPP_ */
