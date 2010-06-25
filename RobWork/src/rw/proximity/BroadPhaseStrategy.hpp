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

#ifndef RW_PROXIMITY_BROADPHASEDETECTOR_HPP
#define RW_PROXIMITY_BROADPHASEDETECTOR_HPP

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

#include "CollisionSetup.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/Geometry.hpp>

namespace rw { namespace proximity {



/**
 * @brief describe the interphase of a broad phase proximity strategy or proximity culler.
 *
 *
 */
class BroadPhaseStrategy {
public:

	/**
	 * @brief
	 */
	virtual void reset(const rw::kinematics::State& state) = 0;

	/**
	 * @brief called once before
	 */
	virtual void update(const rw::kinematics::State& state) = 0;

	virtual const rw::kinematics::FramePair& next() = 0;

	virtual bool hasNext() = 0;

	virtual CollisionSetup& getCollisionSetup() = 0;

	virtual void addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom) = 0;

	virtual void removeModel(rw::kinematics::Frame* frame, const std::string& geoid) = 0;

};

typedef rw::common::Ptr<BroadPhaseStrategy> BroadPhaseStrategyPtr;


}
}

#endif /* RW_PROXIMITY_BROADPHASEDETECTOR_HPP */
