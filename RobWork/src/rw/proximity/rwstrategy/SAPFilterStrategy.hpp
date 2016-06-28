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

#ifndef RW_PROXIMITY_SAPFilterStrategy_HPP_
#define RW_PROXIMITY_SAPFilterStrategy_HPP_

#include "ProximityFilterStrategy.hpp"
#include "ProximitySetup.hpp"

namespace rw { namespace models { class WorkCell; } }

namespace rw { namespace proximity {

/**
 * @brief This is a Sweep-And-Prune based filter strategy (broadphase strategy).
 *
 */
class SAPFilterStrategy: public ProximityFilterStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<SAPFilterStrategy> Ptr;

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
	SAPFilterStrategy(rw::common::Ptr<rw::models::WorkCell> workcell);

	/**
	 * @brief constructor - constructs frame pairs based on the \b setup
	 * @param workcell [in] the workcell
	 * @param setup [in] the ProximitySetup describing exclude/include relations
	 */
	SAPFilterStrategy(rw::common::Ptr<rw::models::WorkCell> workcell, const ProximitySetup& setup);


	//! @brief destructor
	virtual ~SAPFilterStrategy(){};

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

	/**
	 * @copydoc ProximityFilterStrategy::addGeometry
	 */
	virtual void addGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr);

	/**
	 * @copydoc ProximityFilterStrategy::removeGeometry(rw::kinematics::Frame*, const rw::geometry::Geometry::Ptr)
	 */
	virtual void removeGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr);

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
typedef rw::common::Ptr<SAPFilterStrategy> SAPFilterStrategyPtr;
#endif
}
}

#endif /* SAPFilterStrategy_HPP_ */
