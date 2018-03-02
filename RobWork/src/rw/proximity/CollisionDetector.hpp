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

#ifndef RW_PROXIMITY_COLLISIONDETECTOR_HPP
#define RW_PROXIMITY_COLLISIONDETECTOR_HPP

/**
 * @file CollisionDetector.hpp
 *
 * \copydoc rw::proximity::CollisionDetector
 */

#include "CollisionStrategy.hpp"
#include "ProximityFilterStrategy.hpp"
#include "ProximityStrategyData.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/Timer.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <vector>

namespace rw {
namespace kinematics {
class Frame;
class State;
}
}

namespace rw { namespace models { class WorkCell; } }

namespace rw {
namespace proximity {

/** @addtogroup proximity */
/*@{*/

/**
 @brief The CollisionDetector implements an efficient way of checking a
 complete frame tree for collisions.

 It relies on a BroadPhaseDetector to do initial filtering which removes obviously not
 colliding frame pairs.

 After the filtering the remaining frame pairs are tested for collision using an
 CollisionStrategy which is a narrow phase collision detector.

 The collision detector does not dictate a specific detection
 strategy or algorithm, instead it relies on the CollisionStrategy interface for
 the actual collision checking between two frames.

 @note The collision detector is not thread safe and as such should not be used by multiple
 threads at a time.
 */
class CollisionDetector
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<CollisionDetector> Ptr;
	//! @brief smart pointer type to this const class
	typedef rw::common::Ptr< const CollisionDetector > CPtr;

    //! @brief types of collision query
    typedef enum
    {
        AllContactsFullInfo, //! find all collisions and return full collision information eg. CollisionStrategy::AllContact
        AllContactsNoInfo, //! find all collisions but without collision information eg. CollisionStrategy::FirstContact
        FirstContactFullInfo,//! return on first contact and include full collision information eg. CollisionStrategy::AllContact
        FirstContactNoInfo //! return on first collision but without collision information eg. CollisionStrategy::FirstContact
    } QueryType;


	/**
	 * @brief result of a collision query
	 */
	struct QueryResult
	{
	    //! the frames that are colliding
	    kinematics::FramePairSet collidingFrames;

	    //! for keeping track of all collision data: AllContactsFullInfo, FirstContactNoInfo
	    std::vector<ProximityStrategyData> _fullInfo;
	};

	/**
	 * @brief Collision detector for a workcell with only broad-phase collision checking.
	 *
	 * The default collision setup stored in the workcell is used for
	 * broad phase collision filtering as a static filter list.
	 *
	 * Notice that no narrow phase checking is performed.
	 * If broad-phase filter returns any frame-pairs, this will be taken as a collision.
	 *
	 * @param workcell [in] the workcell.
	 */
	CollisionDetector(rw::common::Ptr<rw::models::WorkCell> workcell);

	/**
	 * @brief Collision detector for a workcell.
     *
     * The collision detector is initialized with the \b strategy .
     * Notice that the collision detector will create and store models inside the \b strategy .
     *
     * The default collision setup stored in the workcell is used for
     * broad phase collision filtering as a static filter list.
     *
	 * @param workcell [in] the workcell.
	 * @param strategy [in/out] the strategy for narrow-phase checking. The strategy will have models added to it.
	 */
	CollisionDetector(rw::common::Ptr<rw::models::WorkCell> workcell, CollisionStrategy::Ptr strategy);

    /**
     * @brief Collision detector for a workcell.
     * Collision checking is done for the provided collision setup alone.
     *
     * @param workcell [in] the workcell.
     * @param strategy [in/out] the strategy for narrow-phase checking. The strategy will have models added to it.
     * @param filter [in] proximity filter used to cull or filter frame-pairs that are obviously not colliding
     */
	CollisionDetector(rw::common::Ptr<rw::models::WorkCell> workcell,
		CollisionStrategy::Ptr strategy,
		ProximityFilterStrategy::Ptr filter);

#if __cplusplus >= 201103L
	//! @brief Copy constructor is non-existent. Copying is not possible!
    CollisionDetector(const CollisionDetector&) = delete;

	//! @brief Assignment operator is non-existent. Copying is not possible!
    CollisionDetector& operator=(const CollisionDetector&) = delete;
#endif

    /**
     @brief Check the workcell for collisions.
     @param state [in] The state for which to check for collisions.
     @param result [out] If non-NULL, the pairs of colliding frames are
     inserted in \b result.
     @param stopAtFirstContact [in] If \b result is non-NULL and \b
     stopAtFirstContact is true, then only the first colliding pair is
     inserted in \b result. By default all colliding pairs are inserted.
     @return true if a collision is detected; false otherwise.
     */
    bool inCollision(const kinematics::State& state, QueryResult* result = 0, bool stopAtFirstContact = false) const;

    /**
     @brief Check the workcell for collisions.
     @param state [in] The state for which to check for collisions.
     @param data [in/out] Defines parameters for the collision check, the results and also
     enables caching inbetween calls to incollision
     @return true if a collision is detected; false otherwise.
     */
    bool inCollision(const kinematics::State& state, class ProximityData &data) const;

    /**
     * @brief The broad phase collision strategy of the collision checker.
     */
	ProximityFilterStrategy::Ptr getProximityFilterStrategy() const
    {
        return _bpfilter;
    }

    /**
     * @brief Get the narrow-phase collision strategy.
     * @return the strategy if set, otherwise NULL.
     */
	CollisionStrategy::Ptr getCollisionStrategy() const
    {
        return _npstrategy;
    }

	/**
	 * @brief Add Geometry associated to \b frame
	 * 
	 * The current shape of the geometry is copied, hence later changes to \b geometry has no effect
	 *
	 * @param frame [in] Frame to associate geometry to
	 * @param geometry [in] Geometry to add
	 */
	void addGeometry(rw::kinematics::Frame* frame, const rw::common::Ptr<rw::geometry::Geometry> geometry);

	/**
	 * @brief Removes geometry from CollisionDetector
	 * 
	 * The id of the geometry is used to match the collision model to the geometry.
	 *
	 * @param frame [in] The frame which has the geometry associated
	 * @param geometry [in] Geometry with the id to be removed
	 */
	void removeGeometry(rw::kinematics::Frame* frame, const rw::common::Ptr<rw::geometry::Geometry> geometry);
	
	/**
	 * @brief Removes geometry from CollisionDetector
	 * 
	 * The \b geometryId is used to match the collision model to the geometry.
	 *
	 * @param frame [in] The frame which has the geometry associated
	 * @param geometryId [in] Id of geometry to be removed
	 */
	void removeGeometry(rw::kinematics::Frame* frame, const std::string geometryId);

	//! @brief Adds rule specifying inclusion/exclusion of frame pairs in collision detection
	void addRule(const ProximitySetupRule& rule);
	
	//! @brief Removes rule specifying inclusion/exclusion of frame pairs in collision detection
	void removeRule(const ProximitySetupRule& rule);

	/**
	 * @brief Get the computation time used in the inCollision functions.
	 * @return the total computation time.
	 */
	double getComputationTime() const {
		return _timer.getTime();
	}

	/**
	 * @brief Get the number of times the inCollision functions have been called.
	 * @return number of calls to inCollision functions.
	 */
	int getNoOfCalls() const {
		return _numberOfCalls;
	}

	/**
	 * @brief Reset the counter for inCollision invocations and the computation timer.
	 */
	void resetComputationTimeAndCount() {
		_timer.resetAndPause();
		_numberOfCalls = 0;
	}

    /**
     * @brief return the ids of all the geometries of this frames.
     */
    std::vector<std::string> getGeometryIDs(rw::kinematics::Frame *frame);

	/**
	 * @brief Returns whether frame has an associated geometry with \b geometryId.
	 * @param frame [in] Frame in question
	 * @param geometryId [in] Id of the geometry
	 */
	bool hasGeometry(rw::kinematics::Frame* frame, const std::string& geometryId);

private:
	/**
	 * @brief Initialize the narrow phase strategy for the given workcell.
	 * @param wc [in] the workcell.
	 */
    void initialize(rw::common::Ptr<rw::models::WorkCell> wc);

    //! @brief Timer for measuring the time spent in inCollision functions.
	mutable rw::common::Timer _timer;
	//! @brief The number of calls to the inCollision functions.
	mutable int _numberOfCalls;
    //! @brief the broad phase collision strategy
	ProximityFilterStrategy::Ptr _bpfilter;
    //! @brief the narrow phase collision strategy
	CollisionStrategy::Ptr _npstrategy;
	//! @brief Map from frame to collision model.
	rw::kinematics::FrameMap<ProximityModel::Ptr> _frameToModels;

#if __cplusplus < 201103L
private:
    CollisionDetector(const CollisionDetector&);
    CollisionDetector& operator=(const CollisionDetector&);
#endif
};

/*@}*/
}
} // end namespaces

#endif // end include guard
