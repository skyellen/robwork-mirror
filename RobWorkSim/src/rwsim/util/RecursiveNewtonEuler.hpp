/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_UTIL_RECURSIVENEWTONEULER_HPP_
#define RWSIM_UTIL_RECURSIVENEWTONEULER_HPP_

/**
 * @file RecursiveNewtonEuler.hpp
 *
 * \copydoc rwsim::util::RecursiveNewtonEuler
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/InertiaMatrix.hpp>

#include <vector>

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class JointDevice; } }
namespace rwsim { namespace dynamics { class RigidDevice; } }

namespace rwsim {
namespace util {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief The recursive Newton-Euler method is used for calculating inverse dynamics of a kinematic tree.
 */
class RecursiveNewtonEuler {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<RecursiveNewtonEuler> Ptr;

	/**
	 * @brief Constructor for inverse dynamics for a RigidDevice
	 * @param device [in] a rigid device
	 */
	RecursiveNewtonEuler(rw::common::Ptr<rwsim::dynamics::RigidDevice> device);

	/**
	 * @brief Destructor
	 */
	virtual ~RecursiveNewtonEuler();

	/**
	 * @brief Get the used gravity in base coordinates.
	 * @return gravity vector
	 */
	rw::math::Vector3D<> getGravity() const;

	/**
	 * @brief Set the gravity used
	 * @param gravity [in] vector giving the gravity in base coordinates.
	 */
	void setGravity(const rw::math::Vector3D<> &gravity);

	/**
	 * @brief Get the center of mass of the payload in the tool frame.
	 * @return center of mass of the payload
	 */
	rw::math::Vector3D<> getPayloadCOM() const;

	/**
	 * @brief Get the mass of the payload.
	 * @return mass of payload in end frame of device.
	 */
	double getPayloadMass() const;

	/**
	 * @brief Get the inertia of the payload.
	 * @return inertia of payload in end frame of device.
	 */
	rw::math::InertiaMatrix<> getPayloadInertia() const;

	/**
	 * @brief Set the payload relative to the end frame of the device
	 * @param com [in] the center of mass in the end frame.
	 * @param payload [in] the mass of the payload
	 * @param inertia [in] the inertia of the payload
	 */
	void setPayload(const rw::math::Vector3D<> &com, double payload, const rw::math::InertiaMatrix<> &inertia);

	/**
	 * @brief Get the force and torque that the robot exerts on the environment at the TCP frame in base coordinates.
	 * @return a 6D wrench
	 */
	rw::math::Wrench6D<> getEnvironment() const;

	/**
	 * @brief Set the force and torque that the robot exerts on the environment at the TCP frame in base coordinates.
	 * @param wrench [in] the wrench the robot exerts on the environment
	 */
	void setEnvironment(const rw::math::Wrench6D<> &wrench);

	/**
	 * @brief Motion of a body defined as velocity and acceleration.
	 */
	struct Motion {
		rw::math::VelocityScrew6D<> velocity;		//! Velocity
		rw::math::VelocityScrew6D<> acceleration;	//! Acceleration
	};

	/**
	 * @brief Get the Motion of each link and the payload for a device that have accelerating joints.
	 * @param state [in] the state of the device
	 * @param dq [in] joint speeds
	 * @param ddq [in] joint accelerations
	 * @return vector of velocities and accelerations of the center of mass of each body in base coordinates
	 */
	std::vector<Motion> getBodyMotion(const rw::kinematics::State &state, const rw::math::Q &dq, const rw::math::Q &ddq) const;

	/**
	 * @brief Determine the net force and torque acting on each link to achieve the given motion.
	 * @param motions [in] the motion of each link
	 * @param state [in] state (position) of the robot
	 * @return the net forces and torques acting on each link as a vector
	 */
	std::vector<rw::math::Wrench6D<> > getBodyNetForces(const std::vector<Motion> &motions, const rw::kinematics::State &state) const;

	/**
	 * @brief Determine the forces and torques acting in each joint to achieve the given net forces and torques on the links and payload.
	 * @param bodyNetForces [in] the desired net forces and torques acting on each link in link local coordinate frames
	 * @param state [in] state (position) of the robot
	 * @return the net forces and torques acting on each joint in base coordinates.
	 */
	std::vector<rw::math::Wrench6D<> > getJointForces(const std::vector<rw::math::Wrench6D<> > &bodyNetForces, const rw::kinematics::State &state) const;

	/**
	 * @brief Do inverse dynamics for a robot in motion.
	 * @param state [in] state (position) of the robot
	 * @param dq [in] the joint velocities
	 * @param ddq [in] the joint accelerations
	 * @return the wrenches working in each joint in base coordinates
	 */
	std::vector<rw::math::Wrench6D<> > solve(const rw::kinematics::State &state, const rw::math::Q dq = rw::math::Q(), const rw::math::Q ddq = rw::math::Q()) const;

	/**
	 * @brief Get the torque provided by each motor.
	 * @param state [in] state (position) of the robot
	 * @param dq [in] the joint velocities
	 * @param ddq [in] the joint accelerations
	 * @return the torques provided by each motor
	 */
	std::vector<double> solveMotorTorques(const rw::kinematics::State &state, const rw::math::Q &dq, const rw::math::Q &ddq) const;

	/**
	 * @brief Check if solver will work for the given device - if not an exception will be thrown when trying to solve.
	 * @return true if solver works for the given device, false otherwise.
	 */
	bool validate() const;

private:
	static std::string invalidMsg();
	static rw::math::Vector3D<> toVector3D(const rw::math::EAA<> &eaa);

	const rw::common::Ptr<rwsim::dynamics::RigidDevice> _rdev;
	const rw::common::Ptr<rw::models::JointDevice> _jdev;
	rw::math::Vector3D<> _gravity;
	rw::math::Vector3D<> _payloadCOM;
	double _payloadMass;
	rw::math::InertiaMatrix<> _payloadInertia;
	rw::math::Wrench6D<> _environment;
	bool _valid;
};
//! @}
} /* namespace util */
} /* namespace rwsim */
#endif /* RWSIM_UTIL_RECURSIVENEWTONEULER_HPP_ */
