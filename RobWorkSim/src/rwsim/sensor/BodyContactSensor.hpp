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

namespace rwsim {
namespace sensor {
	//! @addtogroup sensor
	//! @{

	/**
	 * @brief This sensor attaches to a body and records all forces and the corresponding
	 * positions where the forces act.
	 *
	 */
	class BodyContactSensor: public SimulatedTactileSensor {
	public:
		//! @brief Smart pointer type for BodyContactSensor.
	    typedef rw::common::Ptr<BodyContactSensor> Ptr;

		/**
		 * @brief constructor
		 * @param name [in] the sensor name
		 * @param frame [in] the frame of this sensor.
		 */
		BodyContactSensor(const std::string& name, rw::kinematics::Frame* frame);

		//! @brief Destructor.
		virtual ~BodyContactSensor();

		//// Interface inherited from SimulatedSensor

		//! @copydoc SimulatedSensor::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedSensor::reset
		void reset(const rw::kinematics::State& state);

		/**
		 * @brief Get sensor (should not be used).
		 * @return NULL always.
		 */
		rw::sensor::Sensor::Ptr getSensor();


		//// Inherited from SimulatedTactileSensor
		void addForceW(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   rw::common::Ptr<rwsim::dynamics::Body> body = NULL);

		void addForce(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   rw::common::Ptr<rwsim::dynamics::Body> = NULL);


        void addWrenchToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
					  rw::common::Ptr<rwsim::dynamics::Body> body=NULL){ };

        void addWrenchWToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
					  rw::common::Ptr<rwsim::dynamics::Body> body=NULL){};


		// now for the functions belonging to this class
		/**
		 * @brief return all contacts registered in the last timestep
		 *
		 * @note all the contacts are represented in body frame.
		 * @param state [in] the state.
		 * @return a vector of contacts.
		 */
		const std::vector<rw::sensor::Contact3D>& getContacts(const rw::kinematics::State& state) const {
			return _sdata.getStateCache<ClassState>(state)->_contacts;
		}

		/**
		 * @brief Get the bodies in contact.
		 * @param state [in] the state.
		 * @return a list of bodies.
		 */
		std::vector<rw::common::Ptr<rwsim::dynamics::Body> > getBodies(const rw::kinematics::State& state){
			return _sdata.getStateCache<ClassState>(state)->_bodies;
		}

	public: // stateless stuff
		//! @brief The state of the sensor.
		class ClassState: public rw::kinematics::StateCache {
		public:
			//! @brief The contacts detected by the sensor (temporary).
			std::vector<rw::sensor::Contact3D> _contactsTmp;
			//! @brief The contacts detected by the sensor.
			std::vector<rw::sensor::Contact3D> _contacts;
			//! @brief The bodies in contact with the sensor (temporary).
			std::vector<rw::common::Ptr<rwsim::dynamics::Body> > _bodiesTmp;
			//! @brief The bodies in contact with the sensor (temporary).
			std::vector<rw::common::Ptr<rwsim::dynamics::Body> > _bodies;

            //! @copydoc StateCache::size
            size_t size() const{
                return (_contacts.size()+_contactsTmp.size())*sizeof(rw::sensor::Contact3D) +
                		(_bodiesTmp.size()+_bodies.size())*sizeof(rw::common::Ptr<rwsim::dynamics::Body>);
            }

            //! @copydoc StateCache::clone
            rw::common::Ptr<rw::kinematics::StateCache> clone() const{
                return rw::common::ownedPtr( new ClassState( *this ) );
            }
		};

	private:
		// hmm,
		rw::math::Transform3D<> _wTf, _fTw;
		rw::kinematics::StatelessData<int> _sdata;
	};

    /**
     * @brief Old smart pointer type.
     * @deprecated Please use BodyContactSensor::Ptr instead.
     */
	typedef rw::common::Ptr<BodyContactSensor> BodyContactSensorPtr;
	//! @}
}
}
#endif /* BODYFORCESENSOR_HPP_ */
