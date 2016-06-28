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
#include "ProximitySetup.hpp"
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameMap.hpp>

namespace rw { namespace models { class WorkCell; } }

namespace rw { namespace proximity {

/**
 * @brief a simple rule based broadphase filter strategy. A static frame pair list of
 * frame pairs that is to be checked for collision is maintained. The list is static in
 * the sense that it is not optimized to be changed. However the user can both add and remove
 * new geometries and rules.
 *
 * @note The framepair list is explicitly kept in this class which makes this broadphase strategy
 * infeasible for workcells with many objects. Consider a workcell with 100 objects, this
 * will in worst case make a list of 10000 framepairs.
 */
class BasicFilterStrategy: public ProximityFilterStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<BasicFilterStrategy> Ptr;

private:
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

public:

	/**
	 * @brief constructor - the ProximitySetup will be extracted from
	 * the workcell description if possible. 
	 *
	 * @param workcell [in] the workcell.
	 */
	BasicFilterStrategy(rw::common::Ptr<rw::models::WorkCell> workcell);

	/**
	 * @brief constructor - constructs frame pairs based on the \b setup
	 * @param workcell [in] the workcell
	 * @param setup [in] the ProximitySetup describing exclude/include relations
	 */
	BasicFilterStrategy(rw::common::Ptr<rw::models::WorkCell> workcell, const ProximitySetup& setup);


	//! @brief destructor
	virtual ~BasicFilterStrategy(){};

	//////// interface inherited from BroadPhaseStrategy

	//! @copydoc ProximityFilterStrategy::reset
	virtual void reset(const rw::kinematics::State& state);

	//! @copydoc ProximityFilterStrategy::createProximityCache
	virtual ProximityCache::Ptr createProximityCache(){ return rw::common::ownedPtr(new Cache(this)); }

	//! @copydoc ProximityFilterStrategy::update
	virtual ProximityFilter::Ptr update(const rw::kinematics::State& state);

	//! @copydoc ProximityFilterStrategy::createProximityCache
	virtual ProximityFilter::Ptr update(const rw::kinematics::State& state, ProximityCache::Ptr data);

	/**
	 * @copydoc BroadPhaseStrategy::getProximitySetup
	 */
	ProximitySetup& getProximitySetup();

	#ifdef RW_USE_DEPRECATED

    /**
     * @brief adds the \b framepair to the framelist
     */
    void include(const kinematics::FramePair& framepair);

    /**
     * @brief adds all possible framepairs containing the frame \b frame to the framelist
     */
    void include(kinematics::Frame* frame, const std::vector<kinematics::Frame*>& frames);

    /**
     * @brief removes the \b framepair from the framepair list
     */
    void exclude(const kinematics::FramePair& framepair);

    /**
     * @brief removes all possible framepairs containing the frame \b frame from the framelist
     */
    void exclude(kinematics::Frame* frame);

	/**
	 * @copydoc BroadPhaseStrategy::addModel
	 */
	std::string addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom);

	/**
	 * @copydoc BroadPhaseStrategy::removeModel
	 */
	void removeModel(rw::kinematics::Frame* frame, const std::string& geoid);

#endif //#ifdef RW_USE_DEPRECATED

	/**
	 * @copydoc ProximityFilterStrategy::addGeometry
	 */
	virtual void addGeometry(rw::kinematics::Frame* frame, const rw::common::Ptr<rw::geometry::Geometry>);

	/**
	 * @copydoc ProximityFilterStrategy::removeGeometry(rw::kinematics::Frame*, const rw::common::Ptr<rw::geometry::Geometry>)
	 */
	virtual void removeGeometry(rw::kinematics::Frame* frame, const rw::common::Ptr<rw::geometry::Geometry>);

	/**
	 * @copydoc ProximityFilterStrategy::removeGeometry(rw::kinematics::Frame*, const std::string&)
	 */
	virtual void removeGeometry(rw::kinematics::Frame* frame, const std::string& geometryId);

	/**
	 * @copydoc ProximityFilterStrategy::addRule
	 */
	virtual void addRule(const ProximitySetupRule& rule);

	/**
	 * @copydoc ProximityFilterStrategy::removeRule
	 */
	virtual void removeRule(const ProximitySetupRule& rule);


private:

	rw::common::Ptr<rw::models::WorkCell> _workcell;
    ProximitySetup _psetup;
	kinematics::FramePairSet _collisionPairs;
	
	kinematics::FrameMap<std::vector<std::string> > _frameToGeoIdMap;

	void applyRule(const ProximitySetupRule& rule, rw::common::Ptr<rw::models::WorkCell> workcell, rw::kinematics::FramePairSet& result);
	void initialize();
	void initializeCollisionFramePairs(const rw::kinematics::State& state);
};

#ifdef RW_USE_DEPREACTED
typedef rw::common::Ptr<BasicFilterStrategy> BasicFilterStrategyPtr;
#endif
}
}

#endif /* BasicFilterStrategy_HPP_ */
