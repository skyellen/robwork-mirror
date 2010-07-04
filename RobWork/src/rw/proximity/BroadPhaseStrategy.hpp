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
 * @brief describe the interface of a broad phase proximity strategy or proximity culler.
 *
 * A broadphase strategy implement heuristics or rules for finding frame pairs that
 * are possibly overlapping and excluding framepairs that are definitely not overlapping.
 *
 * The interface supports early exiting by returning frame-pairs in an iterative manor. This
 * enables efficient collision filtering at the cost of ease of use. Before acquiring sets of
 * framepairs the update function need be called. Thereafter multiple calls to next
 * will return possibly colliding frame pairs.
 *
 * \code
 * bpstrategy->update(state)
 * while(bpstrategy->hasNext()){
 *  FramePair fpair = bpstrategy->next();
 * 	// do collision with narrowphase strategy
 *  ...
 * }
 * \endcode
 *
 */
class BroadPhaseStrategy {
public:

	/**
	 * @brief reset
	 */
	virtual void reset(const rw::kinematics::State& state) = 0;

	/**
	 * @brief called once before acquirering all possibly colliding
	 * frame pairs in the workcell
	 * @param state [in] the state for which collision detection is performed.
	 */
	virtual void update(const rw::kinematics::State& state) = 0;

	/**
	 * @brief returns the next possibly colliding framepair.
	 * @return a frame pair
	 */
	virtual const rw::kinematics::FramePair& next() = 0;

	/**
	 * @brief if there are any more possibly colliding framepairs since last
	 * call to update then this will return true, else false will be returned.
	 */
	virtual bool hasNext() = 0;

	/**
	 * @brief get the collision setup that describe the include/exclude relations of this
	 * BroadPhaseStrategy
	 */
	virtual CollisionSetup& getCollisionSetup() = 0;

	/**
	 * @brief this will associate a model (based on the geometry) with the \b frame.
	 */
	virtual std::string addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom) = 0;

	/**
	 * @brief removes the geometric model with id \b geoid associated with
	 * Frame \b frame from this strategy.
	 */
	virtual void removeModel(rw::kinematics::Frame* frame, const std::string& geoid) = 0;

};

typedef rw::common::Ptr<BroadPhaseStrategy> BroadPhaseStrategyPtr;


}
}

#endif /* RW_PROXIMITY_BROADPHASEDETECTOR_HPP */
