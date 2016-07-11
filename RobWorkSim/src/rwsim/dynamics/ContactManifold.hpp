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

#ifndef RWSIM_DYNAMICS_CONTACTMANIFOLD_HPP_
#define RWSIM_DYNAMICS_CONTACTMANIFOLD_HPP_

//! @file ContactMAnifold.hpp

#include "ContactPoint.hpp"

namespace rwsim {
namespace dynamics {
	class ContactPoint;

	/**
	 * @brief A contact manifold is an area of contact between two objects,
	 * that exists for several timesteps in a simulated environment.
	 * For each timestep the manifold is updated with new contact points such
	 * that the manifold is dynamic
	 *
	 * for each timestep do the following:
	 *
	 *
	 * manifold.reset();
	 *
	 *
	 */
	class ContactManifold {
	public:

		/**
		 * @breif default constructor
		 * @return
		 */
		ContactManifold(){};

		/**
		 * @brief destructor
		 * @return
		 */
		virtual ~ContactManifold(){};


		/**
		 * @brief update the manifold, which means removing contact
		 * points that no longer are valid.
		 */
		void update(){};

		/**
		 * @brief test if the contactpoint \b point is close enough to the
		 * manifold to be part of it.
		 * @param point [in]
		 * @return true if point is close enough to manifold, false otherwise
		 */
		bool isInManifold(ContactPoint& point){return true;};

		/**
		 * @brief add a contact point to this contact manifold.
		 * @param point [in] the
		 */
		void addContact(ContactPoint& point){};

		/**
		 * @brief updates the \b manifolds with the new contact points.
		 * if some contact points does not fit in one of the manifolds then
		 * new manifolds are added.
		 * @param points
		 * @param manifolds
		 */
		static void generateContactManifolds(
			std::vector<ContactPoint*>& points,
			std::vector<ContactManifold*>& manifolds);

		/**
		 * @brief updates the \b manifolds with the new contact points.
		 * if some contact points does not fit in one of the manifolds then
		 * new manifolds are added.
		 *  The clustering method for this contact generator is based on a simple
		 * max distance from deepest penetrating point.
		 * @param points
		 * @param manifolds
		 */
		static void genThresContactManifolds(
			std::vector<ContactPoint>& points,
			std::vector<ContactManifold*>& manifolds,
			double thres);

	private:
		ContactPoint _deepest[5];

	};

}
}
#endif /* CONTACTMANIFOLD_HPP_ */
