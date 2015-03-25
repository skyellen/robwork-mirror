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

#ifndef RWSIM_SENSOR_SIMULATEDTACTILESENSOR_HPP_
#define RWSIM_SENSOR_SIMULATEDTACTILESENSOR_HPP_

#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rw/kinematics/State.hpp>

namespace rwsim {
namespace sensor {
	//! @{
	/**
	 * @brief interface for simulated tactile sensors
	 */
	class SimulatedTactileSensor: public rwlibs::simulation::SimulatedSensor {
	protected:
	    SimulatedTactileSensor(rw::sensor::SensorModel::Ptr model);

	public:

	    //! destructor
	    virtual ~SimulatedTactileSensor(){}

		/**
		 * @brief add a force to a point on the sensor geometry. The force is described
		 * relative to the world frame.
		 * @param point [in] the point where the force is acting.
		 * @param force [in] the direction in which the force is acting
		 * @param cnormal [in] the contact normal where the origin is on the
		 * contacting body and the direction is toward the sensor
		 * @param body [in] the body that caused the contact force. If no body
		 * caused the force on the sensor (could be user input) then the body is NULL
		 */
		virtual void addForceW(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   dynamics::Body::Ptr body = NULL) = 0;

		/**
		 * @brief add a force to a point on the sensor geometry. The force is described
		 * relative to the sensor frame.
		 * @param point [in] the point where the force is acting.
		 * @param force [in] the direction in which the force is acting
		 * @param cnormal [in] the contact normal where the origin is on the
		 * contacting body and the direction is toward the sensor
		 * @param body [in] the body that caused the contact force. If no body
		 * caused the force on the sensor (could be user input) then the body is NULL
		 */
		virtual void addForce(const rw::math::Vector3D<>& point,
					  const rw::math::Vector3D<>& force,
					  const rw::math::Vector3D<>& cnormal,
					  rw::kinematics::State& state,
					  dynamics::Body::Ptr body=NULL) = 0;

		/**
		 * @brief add a wrench to the center of mass of this object
		 * @param force
		 * @param torque
		 * @param state
		 * @param body
		 */
        virtual void addWrenchToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL) = 0;

        /**
         * @brief add a wrench described in World frame to the center of mass of this object
         * @param force
         * @param state
         * @param body
         */
        virtual void addWrenchWToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL) = 0;

	};
	//! @}
}
}
#endif /* SIMULATEDTACTILESENSOR_HPP_ */
