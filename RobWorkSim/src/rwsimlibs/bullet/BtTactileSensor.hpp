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

#ifndef RWSIMLIBS_BULLET_BTTACTILESENSOR_HPP_
#define RWSIMLIBS_BULLET_BTTACTILESENSOR_HPP_

/**
 * @file BtTactileSensor.hpp
 *
 * \copydoc rwsimlibs::bullet::BtTactileSensor
 */

#include <rw/common/Ptr.hpp>
#include <rwlibs/simulation/Simulator.hpp>

#include <vector>

// Forward declarations
class btPersistentManifold;

namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace sensor { class SimulatedTactileSensor; } }

namespace rwsimlibs {
namespace bullet {

class BtBody;
class BtConstraint;

//! @addtogroup rwsimlibs_bullet
//! @{
/**
 * @brief Class for updating SimulatedTactileSensor from Bullet simulation.
 *
 * Note that currently only SimulatedFTSensor is supported, and the sensor measures constraint forces.
 */
class BtTactileSensor {
public:
	/**
	 * @brief Create new tactile sensor.
	 * @param sensor [in] the RobWork sensor.
	 */
	BtTactileSensor(rw::common::Ptr<rwsim::sensor::SimulatedTactileSensor> sensor);

	//! @brief Destructor.
	virtual ~BtTactileSensor();

	/**
	 * @brief Add the feedbacks from constraints that have been added in addFeedback.
	 * @param info [in] information about the simulation update (not used currently).
	 * @param state [in/out] the state to update with new sensor information.
	 */
    void addConstraintsFeedback(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) const;

    /**
     * @brief Add feedback from contacts.
     * @param info [in] information about time-step (used to transform from impulse to force).
     * @param state [in/out] the state to add information to.
     * @param manifold [in] the contact manifold with contact information from bullet solver.
     * @param bodyA [in] the first BtBody.
     * @param bodyB [in] the second BtBody.
     */
    void addContactManifold(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state, const btPersistentManifold* manifold, const BtBody* bodyA, const BtBody* bodyB) const;

    /**
     * @brief Add a constraint that should be measured by the sensor.
     * @param constraint [in] the constraint to get feedback from.
     */
    void addFeedback(BtConstraint* constraint);

private:
    rw::common::Ptr<rwsim::sensor::SimulatedTactileSensor> _rwSensor;
    std::vector<BtConstraint*> _constraints;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTTACTILESENSOR_HPP_ */
