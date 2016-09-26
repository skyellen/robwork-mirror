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

#ifndef RWSIM_DYNAMICS_CONTACTPOINT_HPP_
#define RWSIM_DYNAMICS_CONTACTPOINT_HPP_

//! @file ContactPoint.hpp

#include <rw/math/Vector3D.hpp>
#include <rw/math/InertiaMatrix.hpp>


namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

	/**
	 * @brief representation of a contact point
	 */
	class ContactPoint
	{
	public:
		//! @brief Construct empty contact point.
		ContactPoint():
			isFirstContact(true),
			nImpulse(0.0),
			tImpulse(0.0),
			dist(100),
			bias(0),
			penetrationA(0.0),
			penetrationB(0.0),
			nForce(0),
			tForce(0),
			penetration(0.0),
			K(1.0,1.0,1.0),
			KInv(1.0,1.0,1.0),
			userdata(0),
			mu(0.4)
		{};

		//! @brief Destructor.
		virtual ~ContactPoint(){}

		//! @brief true if first contact
		bool isFirstContact;

		//! @brief normal impulse
		double nImpulse;
		//! @brief tangential impulse
		double tImpulse;

		//! @brief distance between pA and pB
		double dist;
		//! @brief bias
		double bias;
		//! @brief the penetration depth of A into B
		double penetrationA;
		//! @brief the penetration depth of B into A
		double penetrationB;

		//! @brief the normal force
		double nForce;
		//! @brief the normal torque
		double tForce;

		//! @brief the penetration of the contact point
		double penetration;

		//! @brief the contact position
		rw::math::Vector3D<> p;
		//! @brief the contact normal
		rw::math::Vector3D<> n;
		//! @brief the tangential unit velocity
		rw::math::Vector3D<> t;

		//! @brief position of contact point on core of A
		rw::math::Vector3D<> pA;
		//! @brief position pA recorded on first contact
		rw::math::Vector3D<> pAInit;
		//! @brief position of contact point on core of B
		rw::math::Vector3D<> pB;
		//! @brief position pB recorded on first contact
		rw::math::Vector3D<> pBInit;

		// these variables are not generel and therefore should not be here
		//! @brief K
		rw::math::InertiaMatrix<> K;
		//! @brief KInv
		rw::math::InertiaMatrix<> KInv;

		//! @brief user specific data
		void *userdata;

		//! @brief friction
		double mu;


	};
	//! @}
}
}
#endif /*CONTACTPOINT_HPP_*/
