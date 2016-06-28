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

#ifndef RWSIM_UTIL_SPHEREPOSESAMPLER_HPP_
#define RWSIM_UTIL_SPHEREPOSESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>

namespace rwsim {
namespace util {

	/**
	 * @brief samples poses of a movable frame, such that the frame is always
	 * positioned on a sphere around some specified center. Random deviations
	 * to the position of the frame can be added.
	 *
	 * This StateSampler will never become empty.
	 *
	 *
	 *
	 */
	class SpherePoseSampler: public StateSampler
	{
	public:

		/**
		 * @brief constructor \b frame will be set in random poses on
		 * the sphere with center wPc and radius \b radi
		 * @param mframe [in] the frame
		 * @param wPc [in] the origo of the sphere
		 * @param radi [in] radius of sphere
		 */
		SpherePoseSampler(rw::kinematics::MovableFrame* mframe,
						  const rw::math::Vector3D<>& wPc,
						  double radi);

		/**
		 * @brief constructor \b frame will be set in random poses on
		 * the sphere with center wPc and radius implicitly defined as
		 * the distance from \b frame to
		 * \b wPc in \b initState
		 * @param mframe [in] the frame
		 * @param wPc [in] the origo of the sphere
		 * @param initState [in] the initial state
		 */
		SpherePoseSampler(rw::kinematics::MovableFrame* mframe,
						  const rw::math::Vector3D<>& wPc,
						  rw::kinematics::State& initState);

		/**
		 * @brief destructor
		 */
		virtual ~SpherePoseSampler();

		/**
		 * @brief sets the bounds of the rotation noise(small random deviation)
		 * that is added to the generated pose of the frame.
		 * @param low [in] lower bound RPY values
		 * @param upper [in] upper bound RPY values
		 */
		void setRPYNoiseBound(const rw::math::RPY<>& low, const rw::math::RPY<>& upper);

		/**
		 * @brief sets the bounds of the position noise(small random deviation)
		 * that is added to the generated pose of the frame.
		 * @param low [in] lower bound position values
		 * @param upper [in] upper bound position values
		 */
		void setPosNoiseBound(const rw::math::Vector3D<>& low, const rw::math::Vector3D<>& upper);

		//! @copydoc StateSampler::sample
		bool sample(rw::kinematics::State& state);

		//! @copydoc StateSampler::empty
		bool empty() const;

	private:
		rw::kinematics::MovableFrame* _mframe;
		rw::math::Vector3D<> _wPc;
		rw::kinematics::State _initState;
	};
}
}
#endif /* FINITESTATESAMPLER_HPP_ */
