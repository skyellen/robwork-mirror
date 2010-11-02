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

#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/Geometry.hpp>

#include "CollisionSetup.hpp"
#include "ProximityFilter.hpp"
#include "ProximityCache.hpp"

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
 *
 * Filter f = bpstrategy->update(state)
 * while(f->hasNext()){
 *  FramePair fpair = f->next();
 * 	// do collision with narrowphase strategy
 *  ...
 * }
 * \endcode
 *
 */
class ProximityFilterStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ProximityFilterStrategy> Ptr;


	/**
	 * @brief reset
	 */
	virtual void reset(const rw::kinematics::State& state) = 0;

	/**
	 * @brief creates a FilterData object. This is used for caching relavant data between calls to update
	 *
	 * @return
	 */
	virtual ProximityCache::Ptr createProximityCache() = 0;

	/**
	 *
	 * @param state
	 * @return
	 */
	virtual ProximityFilter::Ptr update(const rw::kinematics::State& state) = 0;

	/**
	 * @brief called once before acquirering all possibly colliding
	 * frame pairs in the workcell
	 * @param state [in] the state for which collision detection is performed.
	 */
	virtual ProximityFilter::Ptr update(const rw::kinematics::State& state, ProximityCache::Ptr data) = 0;

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

#ifdef RW_USE_DEPRECATED
typedef rw::common::Ptr<ProximityFilterStrategy> ProximityFilterStrategyPtr;
#endif

}
}

#endif /* RW_PROXIMITY_BROADPHASEDETECTOR_HPP */
