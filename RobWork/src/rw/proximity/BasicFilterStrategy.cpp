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

#include "BasicFilterStrategy.hpp"


#include <rw/models/Models.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/VirtualJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>
#include <map>

using namespace rw;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::models;



BasicFilterStrategy::BasicFilterStrategy(rw::models::WorkCell::Ptr workcell):
	_workcell(workcell),		
	_psetup(ProximitySetup::get(*workcell))
{
	initialize();
	initializeCollisionFramePairs(workcell->getDefaultState());
}

BasicFilterStrategy::BasicFilterStrategy(rw::models::WorkCell::Ptr workcell,const ProximitySetup& setup):
	_workcell(workcell),
	_psetup(setup)
{
	initialize();
	initializeCollisionFramePairs(workcell->getDefaultState());
}


void BasicFilterStrategy::initialize() {
    // run through all objects in workcell and collect the geometric information
    _frameToGeoIdMap.clear();
    State state = _workcell->getDefaultState();
    std::vector<Object::Ptr> objects = _workcell->getObjects();

    BOOST_FOREACH(Object::Ptr object, objects) {
        BOOST_FOREACH(geometry::Geometry::Ptr geom, object->getGeometry(state) ){
            Frame* frame = geom->getFrame();
            RW_ASSERT(frame);
            _frameToGeoIdMap[*frame].push_back(geom->getName());
        }
    }
}
#ifdef RW_USE_DEPRECATED


void BasicFilterStrategy::include(const kinematics::FramePair& framepair)
{
	_collisionPairs.insert(framepair);
}

void BasicFilterStrategy::include(kinematics::Frame* frame, const std::vector<Frame*>& frames)
{
	BOOST_FOREACH(Frame* f, frames) {
		if (f != frame) {
			include(FramePair(frame, f));
		}
	}

}

void BasicFilterStrategy::exclude(const kinematics::FramePair& framepair)
{
	_collisionPairs.erase(framepair);
}

void BasicFilterStrategy::exclude(kinematics::Frame* frame)
{
	FramePairSet::iterator it = _collisionPairs.begin(); 
	while (it != _collisionPairs.end()) {
		if ((*it).first == frame || (*it).second == frame) {
	        // Since post-increment is used, erase() gets a temporary object and
		    // iter has moved beyond the item being erased so it remains valid.
			//it = _collisionPairs.erase(it++);
		    _collisionPairs.erase(it++);
		} else {
			++it;
		}
	}	
}


std::string BasicFilterStrategy::addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom)
{
	include(frame);
	return frame->getName();
}

void BasicFilterStrategy::removeModel(rw::kinematics::Frame* frame, const std::string& geoid)
{
	exclude(frame);
}

#endif //#ifdef RW_USE_DEPRECATED

	//////// interface inherited from BroadPhaseStrategy
void BasicFilterStrategy::reset(const rw::kinematics::State& state)
{

}

//! @copydoc ProximityFilterStrategy::update
ProximityFilter::Ptr BasicFilterStrategy::update(const rw::kinematics::State& state){
	return rw::common::ownedPtr( new BasicFilterStrategy::Filter(_collisionPairs.begin(), _collisionPairs.end() ) );
}

//! @copydoc ProximityFilterStrategy::createProximityCache
ProximityFilter::Ptr BasicFilterStrategy::update(const rw::kinematics::State& state, ProximityCache::Ptr data){
	return rw::common::ownedPtr( new BasicFilterStrategy::Filter(_collisionPairs.begin(), _collisionPairs.end() ) );
}




ProximitySetup& BasicFilterStrategy::getProximitySetup()
{
	return _psetup;
}



void BasicFilterStrategy::addGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geo) {
	if (geo == NULL)
		RW_THROW("Unable to add NULL as geometry");
	if (frame == NULL)
		RW_THROW("Unable to add geometry to NULL frame");

	_frameToGeoIdMap[*frame].push_back(geo->getName());

	initializeCollisionFramePairs(_workcell->getDefaultState());
}

void BasicFilterStrategy::removeGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geo) 
{
	if (geo == NULL)
		RW_THROW("Unable to add NULL as geometry");

	removeGeometry(frame, geo->getName());
	
}

void BasicFilterStrategy::removeGeometry(rw::kinematics::Frame* frame, const std::string& geoName) 
{
	if (frame == NULL)
		RW_THROW("Unable to add geometry to NULL frame");

	std::vector<std::string>& geoNames = _frameToGeoIdMap[*frame];
	for (std::vector<std::string>::iterator it = geoNames.begin(); it != geoNames.end(); ++it) {
		if ((*it)==geoName) {
			geoNames.erase(it);
			break;
		}
	}
	if (geoNames.empty()) {
		_frameToGeoIdMap.erase(*frame);
	}
	initializeCollisionFramePairs(_workcell->getDefaultState());
}



void BasicFilterStrategy::addRule(const ProximitySetupRule& rule) 
{
	_psetup.addProximitySetupRule(rule);

	initializeCollisionFramePairs(_workcell->getDefaultState());
}

void BasicFilterStrategy::removeRule(const ProximitySetupRule& rule) {
	_psetup.removeProximitySetupRule(rule);
	initializeCollisionFramePairs(_workcell->getDefaultState());
	
}



namespace
{
    
    FramePair orderPair(const FramePair& pair)
    {
        if (pair.first->getName() < pair.second->getName())
            return pair;
        else
            return FramePair(pair.second, pair.first);
    }


	void addAllPairs(FrameList frames, FramePairSet& result) {
        typedef FrameList::const_iterator I;
		for (I from = frames.begin(); from != frames.end(); ++from) {
			for (I to = from + 1; to != frames.end(); ++to) {
                result.insert(orderPair(FramePair(*from, *to)));
			}
		}
	}
} 


void BasicFilterStrategy::initializeCollisionFramePairs(const State& state) {
	_collisionPairs.clear();

	FramePairSet result;

	if (_psetup.useIncludeAll()) {
		FrameList allFrames = _workcell->getFrames();
		
		for (FrameList::iterator it = allFrames.begin(); it != allFrames.end(); ) {
			if(*it==NULL)
			    continue;
		    if (!_frameToGeoIdMap.has(*(*it))) {
				it = allFrames.erase(it);
			} else {
				++it;
			}
		}

		addAllPairs(allFrames, result);
	}

	if (_psetup.useExcludeStaticPairs()) {
		std::vector<FrameList> staticGroups = Kinematics::getStaticFrameGroups(_workcell->getWorldFrame(), _workcell->getDefaultState());
		
		FramePairSet exclude_set;
		BOOST_FOREACH(FrameList& group, staticGroups) {
			typedef FrameList::const_iterator I;
			for (I from = group.begin(); from != group.end(); ++from) {
				for (I to = from + 1; to != group.end(); ++to) {
					exclude_set.insert(orderPair(FramePair(*from, *to)));
				}
			}
		}

		BOOST_FOREACH(FramePair fp, exclude_set) {
			FramePairSet::iterator it = result.find(fp);
			if (it != result.end()) {
				result.erase(it);
			}
		}

	}

	BOOST_FOREACH(const ProximitySetupRule& rule, _psetup.getProximitySetupRules()) {
		applyRule(rule, _workcell, result);
	}

	//std::cout<<"Frame Pairs = "<<std::endl;
	//BOOST_FOREACH(FramePair fp, result) {
	//	std::cout<<fp.first->getName()<<" to "<<fp.second->getName()<<std::endl;
	//}

	_collisionPairs.insert(result.begin(), result.end());
}



void BasicFilterStrategy::applyRule(const ProximitySetupRule& rule, WorkCell::Ptr workcell, FramePairSet& result) {
	std::vector<Frame*> frames = workcell->getFrames();
	switch (rule.type()) {
	case ProximitySetupRule::EXCLUDE_RULE: 
		for (FramePairSet::iterator it = result.begin(); it != result.end(); ) {
			if (rule.match((*it).first->getName(), (*it).second->getName())) {
			    FramePairSet::iterator ittmp = it;
			    ++it;
				result.erase(ittmp);
			} else {
				++it;
			}
		}
		break;
	case ProximitySetupRule::INCLUDE_RULE:
		BOOST_FOREACH(Frame* frame1, frames) {
		    if(frame1==NULL)
		        continue;
			if (rule.matchPatternA(frame1->getName())) {
				BOOST_FOREACH(Frame* frame2, frames) {
		            if(frame2==NULL)
		                continue;

				    if (frame1 != frame2) {
						if (rule.matchPatternB(frame2->getName())) {						
							result.insert(FramePair(frame1, frame2));
						}
					}
				}
			}
		}
		break;
	} //end switch
}


