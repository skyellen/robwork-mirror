/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTBroadPhase.hpp"

#include <rw/models/Object.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTBroadPhase::TNTBroadPhase(rw::common::Ptr<const DynamicWorkCell> dwc):
	_dwc(dwc),
	_defaultProximitySetup(getDefaultProximitySetup(dwc)),
	_bpStrategy(getEmptyBPStrategy(dwc)),
	_collisionStrategy(new ProximityStrategyPQP()),
	_frameToBPRule(new FrameMap<std::list<ProximitySetupRule> >()),
	_frameGeoToPModel(new FrameMap<std::map<Geometry::Ptr, ProximityModel::Ptr> >())
{
}

TNTBroadPhase::~TNTBroadPhase() {
	delete _defaultProximitySetup;
	delete _bpStrategy;
	delete _collisionStrategy;
	delete _frameToBPRule;
	delete _frameGeoToPModel;
}

void TNTBroadPhase::addObject(Object::Ptr object) {
	_collisionStrategy->addModel(object);
	const std::vector<ProximitySetupRule>& defaultRules = _defaultProximitySetup->getProximitySetupRules();
	const bool defaultIncludeAll = _defaultProximitySetup->useIncludeAll();
    const std::vector<Geometry::Ptr> geoms = object->getGeometry();
    BOOST_FOREACH(const Geometry::Ptr& geom, geoms){
        const Frame* const geoframe = geom->getFrame();
        if (_frames.find(geoframe) != _frames.end())
        	continue;
        ProximityModel::Ptr pmodel = _collisionStrategy->createModel();
        (*_frameGeoToPModel)[*geoframe][geom] = pmodel;
        _collisionStrategy->addGeometry(pmodel.get(),geom);
        // Find rules that match on the geoframe (no matter if they are include/exclude)
        std::vector<ProximitySetupRule> candidateRules;
        BOOST_FOREACH(const ProximitySetupRule& rule, defaultRules) {
        	if (rule.matchOne(geoframe->getName()))
        		candidateRules.push_back(rule);
        }
        if (defaultIncludeAll || candidateRules.size() > 0) {
        	BOOST_FOREACH(const Frame* frameB, _frames){
        		bool addRule = defaultIncludeAll;
        		// Find the first rule that also match on the second frame.
        		BOOST_FOREACH(const ProximitySetupRule& rule, candidateRules) {
        			if (rule.match(geoframe->getName(),frameB->getName())) {
        				if (rule.type() == ProximitySetupRule::INCLUDE_RULE)
        					addRule = true;
        				else if (rule.type() == ProximitySetupRule::EXCLUDE_RULE)
        					addRule = false;
        				break;
        			}
        		}
        		if (addRule) {
        			const ProximitySetupRule rule = ProximitySetupRule::makeInclude(geoframe->getName(),frameB->getName());
        			_bpStrategy->addRule(rule);
        			(*_frameToBPRule)[*geoframe].push_back(rule);
        			(*_frameToBPRule)[*frameB].push_back(rule);
        		}
        	}
        }
		_frames.insert(geoframe);
	}
}

void TNTBroadPhase::removeObject(Object::Ptr object) {
    std::vector<Geometry::Ptr> geoms = object->getGeometry();
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        Frame* geoframe = geom->getFrame();
        ProximityModel::Ptr model = NULL;
        if (_frameGeoToPModel->has(*geoframe)) {
        	std::map<Geometry::Ptr, ProximityModel::Ptr>& map = (*_frameGeoToPModel)[*geoframe];
        	map.erase(geom);
        	if (map.size() == 0)
        		_frameGeoToPModel->erase(*geoframe);
        }
        if (model->getGeometryIDs().size() == 1) {
            if (_frames.erase(geoframe)!=1)
            	RW_THROW("TNTContactDetector (removeObject): did not erase frame \"" << geoframe->getName() << "\" exactly once in set of frames.");
            if (_frameToBPRule->has(*geoframe)) {
            	BOOST_FOREACH(const ProximitySetupRule& rule, (*_frameToBPRule)[*geoframe]){
            		_bpStrategy->removeRule(rule);
            		BOOST_FOREACH(const Frame* frameB, _frames){
            			(*_frameToBPRule)[*frameB].remove(rule);
            		}
            	}
            	(*_frameToBPRule).erase(*geoframe);
            }
        }
    }
}

TNTBroadPhase::FramePairList TNTBroadPhase::broadPhase(const State& state) {
	std::vector<FramePairConst> pairs;
	const ProximityFilter::Ptr filter = _bpStrategy->update(state);
	while(!filter->isEmpty()){
		const FramePair& pair = filter->frontAndPop();
		pairs.push_back(pair);
	}
	return pairs;
}

bool TNTBroadPhase::maxPenetrationExceeded(rw::common::Ptr<const ContactDetector> detector, const State& state) {
	const FramePairList& frames = broadPhase(state);
	BOOST_FOREACH(const FramePairConst& pair, frames) {
		const Transform3D<> wTa = Kinematics::worldTframe(pair.first,state);
		const Transform3D<> wTb = Kinematics::worldTframe(pair.second,state);
		std::map<Geometry::Ptr, ProximityModel::Ptr>& geoToPModelA = (*_frameGeoToPModel)[*pair.first];
		std::map<Geometry::Ptr, ProximityModel::Ptr>& geoToPModelB = (*_frameGeoToPModel)[*pair.second];
		std::map<Geometry::Ptr, ProximityModel::Ptr>::iterator itA, itB;
		for (itA = geoToPModelA.begin(); itA != geoToPModelA.end(); itA++) {
			for (itB = geoToPModelB.begin(); itB != geoToPModelB.end(); itB++) {
				const Geometry::Ptr geoA = (*itA).first;
				const Geometry::Ptr geoB = (*itB).first;
				const ContactDetector::StrategyTable strategies = detector->getContactStrategies(pair.first->getName(),geoA,pair.second->getName(),geoB);
				if (strategies.size() > 0) {
					if (strategies.front().strategy->getName() != "ContactStrategyPQP")
						continue;
				}
				ProximityModel::Ptr modelA = (*itA).second;
				ProximityModel::Ptr modelB = (*itB).second;
				ProximityStrategyData data;
				data.setCollisionQueryType(CollisionStrategy::FirstContact);
				bool collision = _collisionStrategy->inCollision(modelA,wTa,modelB,wTb,data);
				if (collision) {
					return true;
				}
			}
		}
	}
	return false;
}

ProximityFilterStrategy* TNTBroadPhase::getProximityFilterStrategy() const {
	return _bpStrategy;
}

ProximityFilterStrategy* TNTBroadPhase::getEmptyBPStrategy(rw::common::Ptr<const DynamicWorkCell> dwc) {
	ProximitySetup rules;
	rules.setUseIncludeAll(false);
	return new BasicFilterStrategy(dwc->getWorkcell(),rules);
}

const ProximitySetup* TNTBroadPhase::getDefaultProximitySetup(rw::common::Ptr<const DynamicWorkCell> dwc) {
	BasicFilterStrategy* const bfstrategy = new BasicFilterStrategy(dwc->getWorkcell());
	const ProximitySetup* const psetup = new ProximitySetup(bfstrategy->getProximitySetup());
	delete bfstrategy;
	return psetup;
}
