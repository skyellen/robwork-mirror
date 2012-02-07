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

#ifndef RWSIM_SIMULATOR_ODETACTILESENSOR_HPP_
#define RWSIM_SIMULATOR_ODETACTILESENSOR_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/dynamics/Body.hpp>

#include <vector>
#include <ode/ode.h>

namespace rwsim {
namespace simulator {

	class ODETactileSensor {
	public:
		ODETactileSensor(sensor::SimulatedTactileSensor *sens);

		virtual ~ODETactileSensor(){};

        void addFeedbackGlobal(dJointFeedback*, dynamics::Body* b, int body);

		void addFeedback(const std::vector<dJointFeedback*>& fback, const std::vector<dContactGeom> &g, dynamics::Body* b, int body);

		void clear();

		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);


	//    void setContacts(const rw::proximity::MultiDistanceResult& res,
	//                     rw::math::Transform3D<> wTa,
	//                     rw::math::Transform3D<> wTb);

	private:
		std::vector<rw::math::Transform3D<> > _wTa,_wTb;

		std::vector<dJointFeedback*> _feedbackGlobal;

		std::vector<std::vector<dJointFeedback*> > _feedback;
		std::vector<std::vector<dContactGeom> > _geoms;
		//std::vector<rw::proximity::MultiDistanceResult> _contacts;

		std::vector<dynamics::Body*> _rwBody;
		sensor::SimulatedTactileSensor *_rwsensor;
		//rw::math::Vector3D<> point;
		std::vector<int> _bodyIdx, _bodyGlobalIdx;
		std::vector<dynamics::Body*> _bodyGlobal;

	};
}
}

#endif /* ODETACTILESENSOR_HPP_ */
