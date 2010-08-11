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

#ifndef RW_PROXIMITY_BasicFilterStrategy_HPP_
#define RW_PROXIMITY_BasicFilterStrategy_HPP_

#include "ProximityFilterStrategy.hpp"
#include <rw/models/WorkCell.hpp>
#include "CollisionSetup.hpp"
#include <rw/kinematics/Frame.hpp>

namespace rw { namespace proximity {

/**
 * @brief a simple rule based broadphase filter strategy. A static frame pair list of
 * frame pairs that is to be checked for collision is maintained. The list is static in
 * the sense that it is not optimized to be changed, though the user can both add and remove
 * new geometries and rules.
 *
 * @note The framepair list is explicitly kept in this class which makes this broadphase strategy
 * infeasible for workcells with many objects. Consider a workcell with 100 objects, this
 * will in worst case make a list of 10000 framepairs.
 */
class BasicFilterStrategy: public ProximityFilterStrategy {
public:
	/**
	 * @brief the proximity cache of the basic filter
	 */
	struct Cache: public ProximityCache {
	public:
		Cache(void *owner):ProximityCache(owner){};
		size_t size() const{ return 0;};
		void clear(){};
	};

	struct Filter: public ProximityFilter {
	public:

		Filter(kinematics::FramePairSet::iterator front, kinematics::FramePairSet::iterator end):
			_front(front),_end(end){}

		void pop(){ ++_front; };

		kinematics::FramePair frontAndPop(){
			kinematics::FramePair res = *_front;
			pop();
			return(res);
		}

		rw::kinematics::FramePair front(){ return *_front;};

		bool isEmpty(){ return _front==_end; };

	private:

		kinematics::FramePairSet::iterator _front, _end;

	};

	//! @brief constructor
	BasicFilterStrategy();

	/**
	 * @brief constructor using a set of franes that should describe which frames to test.
	 * @param includeset [in] the set of framepairs that should be testet
	 */
	BasicFilterStrategy(kinematics::FramePairSet includeset);

	/**
	 * @brief constructor - the include/exclude relations will be extracted from
	 * the workcell description if possible. If not a default include relation will
	 * be used.
	 * @param workcell [in] the workcell.
	 */
	BasicFilterStrategy(rw::models::WorkCellPtr workcell);

	/**
	 * @brief constructor - building the include/exclude frampair relations from
	 * the collision setup.
	 * @param workcell [in] the workcell
	 * @param setup [in] the collision setup describing exclude/include relations
	 */
	BasicFilterStrategy(rw::models::WorkCellPtr workcell, const CollisionSetup& setup);

	/**
	 * @brief constructor - building the include/exclude frampair relations from
	 * the collision setup, though with the strategy for filtering out frames that has
	 * no collision geometry.
	 * @param workcell [in] the workcell
	 * @param strategy [in] the collision strategy
	 * @param setup [in] the collision setup describing exclude/include relations
	 */
	BasicFilterStrategy(rw::models::WorkCellPtr workcell, CollisionStrategyPtr strategy, const CollisionSetup& setup);

	//! @brief destructor
	virtual ~BasicFilterStrategy(){};

	/**
	 * @brief adds the \b framepair to the framelist
	 */
	void include(const kinematics::FramePair& framepair);

	/**
	 * @brief adds all possible framepairs containing the frame \b frame to the framelist
	 */
	void include(kinematics::Frame* frame);

	/**
	 * @brief removes the \b framepair from the framepair list
	 */
	void exclude(const kinematics::FramePair& framepair);

	/**
	 * @brief removes all possible framepairs containing the frame \b frame from the framelist
	 */
	void exclude(kinematics::Frame* frame);

	//////// interface inherited from BroadPhaseStrategy

	//! @copydoc ProximityFilterStrategy::reset
	virtual void reset(const rw::kinematics::State& state);

	//! @copydoc ProximityFilterStrategy::createProximityCache
	virtual ProximityCachePtr createProximityCache(){ return rw::common::ownedPtr(new Cache(this)); }

	//! @copydoc ProximityFilterStrategy::update
	virtual ProximityFilterPtr update(const rw::kinematics::State& state);

	//! @copydoc ProximityFilterStrategy::createProximityCache
	virtual ProximityFilterPtr update(const rw::kinematics::State& state, ProximityCachePtr data);

	/**
	 * @copydoc BroadPhaseStrategy::addgetCollisionModel
	 */
	CollisionSetup& getCollisionSetup();

	/**
	 * @copydoc BroadPhaseStrategy::addModel
	 */
	std::string addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom);

	/**
	 * @copydoc BroadPhaseStrategy::removeModel
	 */
	void removeModel(rw::kinematics::Frame* frame, const std::string& geoid);

	//// static methods that manipulate and create frame pair sets

    /**
       @brief The full set of pairs of frames for which to perform collision
       checking when given a workcell \b workcell and a collision setup \b
       setup for the workcell. The collision strategy \b strategy is used to
       verify if a frame has a model of if it can be safely excluded (unless
       other has been specified in \b setup).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
        const rw::models::WorkCell& workcell,
        CollisionStrategy& strategy,
        const CollisionSetup& setup);

    /**
       @brief TODO:Like makeFramePairSet(\b workcell, \b setup) where
       \b setup is the default collision setup registered for the workcell
       (or \b setup is the empty collision setup if no collision setup has
       been specified).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
        const rw::models::WorkCell& workcell,
        const CollisionSetup& setup);

    /**
       @brief Like makeFramePairSet(\b workcell, \b setup, \b setup) where
       \b setup is the default collision setup registered for the workcell
       (or \b setup is the empty collision setup if no collision setup has
       been specified).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
        const rw::models::WorkCell& workcell,
        CollisionStrategy& strategy);

    /**
       @brief Like makeFramePairSet(\b workcell, \b setup) where
       \b setup is the default collision setup registered for the workcell
       (or \b setup is the empty collision setup if no collision setup has
       been specified).
    */
    static
    kinematics::FramePairSet makeFramePairSet(
    		const rw::models::WorkCell& workcell);


    /**
       @brief Assuming that \b device is the only active device, and that
       all other frames are fixed including DAF attachments, return the
       smallest set of pairs of frames that can be deduced to be necessary
       for collision checking.

       The function assumes that DAFs have been attached according to \b
       state.

       Unlike other versions of the makeFramePairSet() function, the
       function *does not* know about the collision setup of the \b workcell
       and *does not* care about DAFs. You may therefore want to take the
       intersection between the set returned here, and the maximum set of
       frames to include in collision checking that has been returned by
       another makeFramePairSet() function.
    */
    static
    kinematics::FramePairSet
    makeFramePairSet(
        const rw::models::Device& device,
        const rw::kinematics::State& state);

    /**
       @brief Pair of (staticSet, dynamicSet) where \b staticSet are the
       pairs of frames to use for collision checking \b controlledDevices
       against the static part of the workcell and \b dynamicSet are the
       pairs of frames to use for dynamically checking \b controlledDevices
       against movement in \b obstacleDevices.

       The construction of (staticSet, dynamicSet) assumes that the state of
       the rest of the workcell is otherwise given by \b state.

       @param workcellSet [in] The standard collision setup for the workcell.
       @param obstacleDevices [in] The set of devices serving as dynamic obstacles.
       @param controlledDevices [in] The set of devices for which planning is done.

       @param state [in] The fixed state relative to which \b
       obstacleDevices and \b controlledDevices move.

       @return (staticSet, dynamicSet) where \b staticSet contain \b
       workcellSet with all pairs removed that reference frames affected by
       \b obstacleDevices, and \b dynamicSet contain all pairs of frames of
       \b workcellSet that (for the same pair) refer to both the frames
       affected by \b obstacleDevices and the frames affected by \b
       controlledDevices.
    */
    static
    std::pair<kinematics::FramePairSet, kinematics::FramePairSet>
    makeStaticDynamicFramePairSet(
        const kinematics::FramePairSet& workcellSet,
        const std::vector<rw::models::DevicePtr>& obstacleDevices,
        const std::vector<rw::models::DevicePtr>& controlledDevices,
        const rw::kinematics::State& state);

    /**
       @brief Write to \b b the intersection of \b a and \b b.

       This is equivalent to erasing from \b b all elements of \b b that are
       not elements of \b a.
    */
    static
    void intersect(const kinematics::FramePairSet& a, kinematics::FramePairSet& b);

    /**
       @brief Write to \b a all elements of \b a that are also elements of
       \b b.
    */
    static
    void subtract(kinematics::FramePairSet& a, const kinematics::FramePairSet& b);

    /**
       @brief Write to \b b the union of the sets \b a and \b b.
    */
    static
    void frameSetUnion(const kinematics::FrameSet& a, kinematics::FrameSet& b);


    static
    kinematics::FramePairList getExcludePairList(const rw::models::WorkCell& workcell,
                                     const CollisionSetup& setup);

private:

	kinematics::FramePairSet _collisionPairs;
	CollisionSetup _csetup;

	// this is the states in this class
	//kinematics::FramePair _pair;
	//kinematics::FramePairSet::iterator _pos;
};

typedef rw::common::Ptr<BasicFilterStrategy> BasicFilterStrategyPtr;
}
}

#endif /* BasicFilterStrategy_HPP_ */
