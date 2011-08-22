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

#ifndef TRWSIM_SENSOR_TACTILEMULTIAXISSIMSENSOR_HPP_
#define TRWSIM_SENSOR_TACTILEMULTIAXISSIMSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/TactileMultiAxisSensor.hpp>

namespace rwsim {
namespace sensor {
	//! @addtogroup sensor
	//! @{

	/**
	 * @brief A sensor that measures force and torque around some reference frame
	 */
	class TactileMultiAxisSimSensor: public rw::sensor::TactileMultiAxisSensor, public SimulatedTactileSensor {
	public:

		/**
		 * @brief constructor
		 * @param name [in] identifier
		 * @param body [in] the body that this sensor is attached to
		 */
		TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body);

		/**
		 * @brief destructor
		 */
		virtual ~TactileMultiAxisSimSensor(){};

		//// Interface inherited from SimulatedSensor
		//! @copydoc SimulatedSensor::update
		void update(double dt, rw::kinematics::State& state);

		//! @copydoc SimulatedSensor::reset
		void reset(const rw::kinematics::State& state);

		//// Interface inherited from SimulatedTactileSensor
		//! @copydoc SimulatedTactileSensor::addForceW
		void addForceW(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   dynamics::Body *body = NULL);

		//! @copydoc SimulatedTactileSensor::addForce
		void addForce(const rw::math::Vector3D<>& point,
					  const rw::math::Vector3D<>& force,
					  const rw::math::Vector3D<>& cnormal,
					  dynamics::Body *body=NULL);


		//! @copydoc TactileMultiAxisSensor::getTransform
		rw::math::Transform3D<> getTransform();

		//!@copydoc TactileMultiAxisSensor::getForce
		rw::math::Vector3D<> getForce();


		//! @copydoc TactileMultiAxisSensor::getTorque
		rw::math::Vector3D<> getTorque();

		//! @copydoc TactileMultiAxisSensor::getMaxTorque
		double getMaxTorque(){return _maxTorque;};


		//! @copydoc TactileMultiAxisSensor::getMaxForce
		double getMaxForce(){return _maxForce;};

	private:
		TactileMultiAxisSimSensor();

	private:
		// the frame that the force and torque is described in relation to
		rw::math::Transform3D<> _transform;
		rw::math::Vector3D<> _force, _torque;
		double _maxForce,_maxTorque;

		//! aux variables updated through \b update
		rw::math::Transform3D<> _wTf, _fTw;
	};
	//! @}
}
}

#endif /* TACTILEMULTIAXISSENSOR_HPP_ */
