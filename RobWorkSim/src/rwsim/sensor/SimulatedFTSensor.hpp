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

#ifndef TRWSIM_SENSOR_SIMULATEDFTSENSOR_HPP_
#define TRWSIM_SENSOR_SIMULATEDFTSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/FTSensor.hpp>
#include <rw/sensor/FTSensorModel.hpp>


namespace rwsim {
namespace sensor {
	//! @addtogroup sensor
	//! @{

	/**
	 * @brief A sensor that measures force and torque between two bodies around
	 * some reference frame.
	 */
	class SimulatedFTSensor: public SimulatedTactileSensor {
	public:
		typedef rw::common::Ptr<SimulatedFTSensor> Ptr;
		/**
		 * @brief constructor - the forces will be described relative to body \b body1
		 * @param name [in] identifier
		 * @param body [in] the first body
		 * @param body1 [in] the second body
		 */
        SimulatedFTSensor(const std::string& name,
                          dynamics::Body::Ptr body,
                          dynamics::Body::Ptr body1,
                          rw::kinematics::Frame* frame=NULL);


		//! @brief destructor
		virtual ~SimulatedFTSensor();

		//// Interface inherited from SimulatedSensor
		//! @copydoc SimulatedSensor::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedSensor::reset
		void reset(const rw::kinematics::State& state);

		//// Interface inherited from SimulatedTactileSensor
		//! @copydoc SimulatedTactileSensor::addForceW
		void addForceW(const rw::math::Vector3D<>& point,
					   const rw::math::Vector3D<>& force,
					   const rw::math::Vector3D<>& cnormal,
					   rw::kinematics::State& state,
					   dynamics::Body::Ptr body = NULL);

		//! @copydoc SimulatedTactileSensor::addForce
		void addForce(const rw::math::Vector3D<>& point,
					  const rw::math::Vector3D<>& force,
					  const rw::math::Vector3D<>& cnormal,
					  rw::kinematics::State& state,
					  dynamics::Body::Ptr body=NULL);

        void addWrenchToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL);

        void addWrenchWToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL);

		//! @copydoc TactileMultiAxisSensor::getTransform
		rw::math::Transform3D<> getTransform() const;

		//!@copydoc TactileMultiAxisSensor::getForce
		rw::math::Vector3D<> getForce(rw::kinematics::State& state) const;


		//! @copydoc TactileMultiAxisSensor::getTorque
		rw::math::Vector3D<> getTorque(rw::kinematics::State& state) const;


		rw::math::Vector3D<> getMaxTorque(){return _ftmodel->getMaxTorque();}

		rw::math::Vector3D<> getMaxForce(){return _ftmodel->getMaxForce();}

		rw::kinematics::Frame * getSensorFrame(){ return _sframe; }

 		void acquire(){}

		 rw::sensor::FTSensor::Ptr getFTSensor(rw::kinematics::State& state);

		 rwsim::dynamics::Body::Ptr getBody1() const { return _body;};
		 rwsim::dynamics::Body::Ptr getBody2() const { return _body1;};
	private:
		SimulatedFTSensor();

	private:
		rwsim::dynamics::Body::Ptr _body, _body1;
		rw::sensor::FTSensor::Ptr _ftsensorWrapper;
		rw::kinematics::Frame *_sframe;
		rw::common::Ptr<rw::sensor::FTSensorModel> _ftmodel;

		struct FTStateData {
			rw::math::Vector3D<> _force, _forceTmp, _torque, _torqueTmp;

			//! aux variables updated through \b update
			rw::math::Transform3D<> _fTb, _wTb, _bTw;
		};

		 rw::kinematics::StatelessData< FTStateData > _sdata;
	};
	//! @}
}
}

#endif /* TACTILEMULTIAXISSENSOR_HPP_ */
