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
#include <rwlibs/simulation/Simulator.hpp>

#include <vector>
#include <ode/ode.h>

namespace rwsim { namespace dynamics { class Body; } }
namespace rwsim { namespace sensor { class SimulatedTactileSensor; } }

namespace rwsim {
namespace simulator {

    /**
     * @brief
     */
	class ODETactileSensor {
	public:
		ODETactileSensor(sensor::SimulatedTactileSensor *sens);

		virtual ~ODETactileSensor(){};

        void clear();

        void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

        void addFeedbackGlobal(dJointFeedback*, rw::common::Ptr<rwsim::dynamics::Body> a, rw::common::Ptr<rwsim::dynamics::Body> b, int body);

		void addFeedback(const std::vector<dJointFeedback*>& fback,
		                 const std::vector<dContactGeom> &g,
						 rw::common::Ptr<rwsim::dynamics::Body> a,
						 rw::common::Ptr<rwsim::dynamics::Body> b,
		                 int body);

		// this is for contacts that are not directly specified by the physics solver, eg. if you emulate
		// multiple contacts with a more complex constraint.
		void addContact(const rw::math::Vector3D<>& pos,
		                const rw::math::Vector3D<>& force,
		                const rw::math::Vector3D<>& normal,
						rw::common::Ptr<rwsim::dynamics::Body> b);

	//    void setContacts(const rw::proximity::MultiDistanceResult& res,
	//                     rw::math::Transform3D<> wTa,
	//                     rw::math::Transform3D<> wTb);

	private:
		std::vector<rw::math::Transform3D<> > _wTa,_wTb;

		std::vector<dJointFeedback*> _feedbackGlobal;

		std::vector<std::vector<dJointFeedback*> > _feedback;
		std::vector<std::vector<dContactGeom> > _geoms;
		//std::vector<rw::proximity::MultiDistanceResult> _contacts;

		std::vector<rw::common::Ptr<rwsim::dynamics::Body> > _rwBody;
		std::vector<int> _bodyFixed;
		sensor::SimulatedTactileSensor *_rwsensor;
		//rw::math::Vector3D<> point;
		std::vector<int> _bodyIdx, _bodyGlobalIdx;
		std::vector<rw::common::Ptr<rwsim::dynamics::Body> > _bodyGlobal;
		std::vector<int> _bodyGlobalFixed;

		struct DirectContact {
		    DirectContact(const rw::math::Vector3D<>& pos,
		                  const rw::math::Vector3D<>& force,
		                  const rw::math::Vector3D<>& normal,
						  rw::common::Ptr<rwsim::dynamics::Body> body):
		        p(pos),f(force),n(normal),b(body)
		    {}
		    rw::math::Vector3D<> p;
		    rw::math::Vector3D<> f;
		    rw::math::Vector3D<> n;
		    rw::common::Ptr<rwsim::dynamics::Body> b;
		};

		std::vector< DirectContact > _directContacts;

	};
}
}

#endif /* ODETACTILESENSOR_HPP_ */
