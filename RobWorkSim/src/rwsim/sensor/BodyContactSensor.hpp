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

#ifndef RWSIM_SENSOR_BODYCONTACTSENSOR_HPP_
#define RWSIM_SENSOR_BODYCONTACTSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/Contact3D.hpp>
#include <rw/sensor/Sensor.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/multi_array.hpp>

namespace rwsim {
namespace sensor {
	//! @addtogroup sensor
	//! @{

	/**
	 * @brief This sensor attaches to a body and records all forces and the corresponding
	 * positions where the forces act.
	 *
	 */
	class BodyContactSensor: public SimulatedTactileSensor, public rw::sensor::Sensor {
	public:

	    typedef rw::common::Ptr<BodyContactSensor> Ptr;

		/**
		 * @brief constructor
		 * @param name [in] the sensor name
		 * @param frame [in] the frame of this sensor.
		 * @return
		 */
		BodyContactSensor(const std::string& name, rw::kinematics::Frame* frame);

		/**
		 * @brief Destructor
		 * @return
		 */
		virtual ~BodyContactSensor();

		//// Interface inherited from SimulatedSensor
		void update(double dt, rw::kinematics::State& state);

		void reset(const rw::kinematics::State& state);

		rw::sensor::Sensor* getSensor();


		//// Inherited from SimulatedTactileSensor
		void addForceW(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   dynamics::Body *body = NULL);

		void addForce(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   dynamics::Body *body = NULL);

		// now for the functions belonging to this class
		/**
		 * @brief return all contacts registered in the last timestep
		 *
		 * @note all the contacts are represented in body frame.
		 *
		 * @return
		 */
		const std::vector<rw::sensor::Contact3D>& getContacts(){
			return _contacts;
		}

		std::vector<rwsim::dynamics::Body*> getBodies(){ return _bodies; }

		rw::kinematics::Frame * getSensorFrame(){ return getFrame(); }

	public: // stateless stuff
		/*
		class ClassState: public rw::kinematics::ObjectStateData {
		public:


		};
		*/

	private:
		// hmm,
		std::vector<rw::sensor::Contact3D> _contactsTmp, _contacts;
		std::vector<rwsim::dynamics::Body*> _bodiesTmp, _bodies;
		rw::math::Transform3D<> _wTf, _fTw;
	};

	typedef rw::common::Ptr<BodyContactSensor> BodyContactSensorPtr;
	//! @}
}
}
#endif /* BODYFORCESENSOR_HPP_ */
