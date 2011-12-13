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

#include "Proximity.hpp"
#include "CollisionSetup.hpp"
#include "CollisionStrategy.hpp"
#include "ProximityFilterStrategy.hpp"
#include "ProximityFilter.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <vector>

namespace rw {
namespace kinematics {
class Frame;
}
}

namespace rw {
namespace proximity {

/** @addtogroup proximity */
/*@{*/

#ifdef RW_USE_DEPRECATED
class CollisionDetector;

//! A pointer to a CollisionDetector.
typedef rw::common::Ptr<CollisionDetector> CollisionDetectorPtr;
#endif

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

    //! @brief types of collision query
    typedef enum
    {
        AllContactsFullInfo, //! find all collisions and return full collision information
        AllContactsNoInfo, //! find all collisions but without collision information
        FirstContactFullInfo,//! return on first contact and include full collision information
        FirstContactNoInfo //! return on first collision but without collision information
    } CollisionQueryType;


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
     @brief Collision detector for a workcell.

     The default collision setup stored in the workcell is used for
     broad phase collision filtering as a static filter list.

     @param workcell [in] the workcell.

     @param strategy [in] the collision checker strategy to use.
     */
	CollisionDetector(rw::models::WorkCell::Ptr workcell);

    /**
     * @brief Collision detector for a workcell
     *
     * The collision dispatcher is initialized with the \b strategy
     *
     * The default collision setup stored in the workcell is used for
     * broad phase collision filtering as a static filter list.
     *
     */
	CollisionDetector(rw::models::WorkCell::Ptr workcell, CollisionStrategy::Ptr strategy);

    /**
     * @brief Collision detector for a workcell.
     * Collision checking is done for the provided collision setup alone.
     *
     * @param workcell [in] the workcell.
     * @param strategy [in] the collision checker strategy to use.
     * @param filter [in] proximity filter used to cull or filter frame-pairs that are obviously not colliding
     */
	CollisionDetector(rw::models::WorkCell::Ptr workcell, 
		CollisionStrategy::Ptr strategy,
		ProximityFilterStrategy::Ptr filter);

    /**
     * @brief set the query type to use as default
     * @param type [in] query type
     */
    void setCollisionQueryType(CollisionQueryType type);

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
     @brief Set the primitive collision strategy to \b strategy.

     \b strategy must be non-NULL.

     @param strategy [in] - the primitive collision checker to use.
     */
    //void setCollisionStrategy(CollisionStrategyPtr strategy, ProximityType ptype);

    /**
     * @brief The collision strategy of the collision checker.
     */
	ProximityFilterStrategy::Ptr getProximityFilterStrategy() const
    {
        return _bpfilter;
    }

    /**
     * @brief get the collision strategy
     */
	CollisionStrategy::Ptr getCollisionStrategy() const
    {
        return _npstrategy;
    }


#ifdef RW_USE_DEPRECATED
    /**
     * @brief adds collision model describing the geometry \b geom. The collision
     * model is associated to the frame.
     */

    void addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom);

    /**
     * @brief adds collision model describing the geometry \b geometry. The collision
     * model is associated to the frame. 
     */
	void addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geometry);

    /**
     * @brief removes a geometry from the specified frame
     */
    void removeModel(rw::kinematics::Frame* frame, const std::string& geoid);


#endif // RW_USE_DEPRECATED

	/**
	 * @brief Add Geometry associated to \b frame
	 * 
	 * The current shape of the geometry is copied, hence later changes to \b geometry has no effect
	 *
	 * @param frame [in] Frame to associated geometry to
	 * @param geometry [in] Geometry to add
	 */
	void addGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geometry);

	/**
	 * @brief Removes geometry from CollisionDetector
	 * 
	 * The id of the geometry is used to match the collision model to the geometry.
	 *
	 * @param frame [in] The frame which has the geometry associated
	 * @param geometry [in] Geometry with the id to be removed
	 */
	void removeGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geometry);
	
	/**
	 * @brief Removes geometry from CollisionDetector
	 * 
	 * The \b geometryId is used to match the collision model to the geometry.
	 *
	 * @param frame [in] The frame which has the geometry associated
	 * @param geometryId [in] Id of geometry to be removed
	 */
	void removeGeometry(rw::kinematics::Frame* frame, const std::string geometryId);


	void addRule(const ProximitySetupRule& rule);
	void removeRule(const ProximitySetupRule& rule);


	double getComputationTime() {
		return _timer.getTime();
	}
	void resetComputationTime() {
		_timer.resetAndPause();
	}

    /**
     * @brief return the ids of all the geometries of this frames.
     */
    std::vector<std::string> getGeometryIDs(rw::kinematics::Frame *frame);

    //void reset(CollisionStrategyPtr strategy);

private:
	mutable rw::common::Timer _timer;

	ProximityFilterStrategy::Ptr _bpfilter;

    // the narrow phase collision strategy
	CollisionStrategy::Ptr _npstrategy;

	rw::kinematics::FrameMap<ProximityModel::Ptr> _frameToModels;

private:
    CollisionDetector(const CollisionDetector&);
    CollisionDetector& operator=(const CollisionDetector&);
};

/*@}*/
}
} // end namespaces

#endif // end include guard
