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


#include "CollisionDetector.hpp"
#include "CollisionStrategy.hpp"
#include "CollisionSetup.hpp"

#include <rw/common/ScopedTimer.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include "BasicFilterStrategy.hpp"

#include "ProximityData.hpp"
#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::geometry;

CollisionDetector::CollisionDetector(WorkCell::Ptr workcell):
        _npstrategy(NULL)
{
	RW_ASSERT(workcell);
	_bpfilter = ownedPtr( new BasicFilterStrategy(workcell) );
}

CollisionDetector::CollisionDetector(WorkCell::Ptr workcell,
									 CollisionStrategy::Ptr strategy) :
    _npstrategy(strategy)
{
    RW_ASSERT(strategy!=NULL);
    RW_ASSERT(workcell!=NULL);
    _bpfilter = ownedPtr( new BasicFilterStrategy(workcell) );

    initialize(workcell);
}

CollisionDetector::CollisionDetector(WorkCell::Ptr workcell,
									 CollisionStrategy::Ptr strategy,
									 ProximityFilterStrategy::Ptr bpfilter) :
    _bpfilter(bpfilter),
    _npstrategy(strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);
    // build the frame map
    initialize(workcell);
}

void CollisionDetector::initialize(WorkCell::Ptr wc) {
    // run through all objects in workcell and collect the geometric information
    std::vector<Object::Ptr> objects = wc->getObjects();
    State state = wc->getDefaultState();
    BOOST_FOREACH(Object::Ptr object, objects) {
        BOOST_FOREACH(geometry::Geometry::Ptr geom, object->getGeometry( state ) ){
            Frame* frame = geom->getFrame();
            RW_ASSERT(frame);
            _npstrategy->addModel(frame, geom);
            _frameToModels[*frame] = _npstrategy->getModel(frame);
        }

    }
}


bool CollisionDetector::inCollision(const kinematics::State& state, ProximityData &proxdata) const
{
    ProximityFilter::Ptr filter = _bpfilter->update(state);
    FKTable fk(state);
    ProximityStrategyData data;

    proxdata._collisionData.collidingFrames.clear();

    bool stopAtFirstContact =
            proxdata.getCollisionQueryType()==FirstContactFullInfo ||
            proxdata.getCollisionQueryType()==FirstContactNoInfo;

    bool fullInfo = proxdata.getCollisionQueryType()==FirstContactFullInfo ||
            proxdata.getCollisionQueryType()==AllContactsFullInfo;

    if(fullInfo ){
        data.setCollisionQueryType( CollisionStrategy::AllContacts );
    } else {
        data.setCollisionQueryType( CollisionStrategy::FirstContact );
    }

    // next we query the BP filter for framepairs that are possibly in collision
    while( !filter->isEmpty() ){
        const FramePair& pair = filter->frontAndPop();

        //std::cout << pair.first->getName() << " " << pair.second->getName() << std::endl;

        // and lastly we use the dispatcher to find the strategy the
        // is required to compute the narrowphase collision
        const ProximityModel::Ptr &a = _frameToModels[*pair.first];
        const ProximityModel::Ptr &b = _frameToModels[*pair.second];

        if(a==NULL || b==NULL)
            continue;

        const Transform3D<> aT = fk.get(*pair.first);
        const Transform3D<> bT = fk.get(*pair.second);
        bool res = _npstrategy->inCollision(a, aT, b, bT, data);
        if( res ){
            proxdata._collisionData.collidingFrames.insert(pair);
            if (stopAtFirstContact)
                return true;
        }
    }

    return proxdata._collisionData.collidingFrames.size()>0;
}

bool CollisionDetector::inCollision(const State& state,
									QueryResult* result,
									bool stopAtFirstContact) const
{
	ScopedTimer stimer(_timer);
	_numberOfCalls++;

	//std::cout << "inCollision" << std::endl;
    // first we update the broadphase filter with the current state
	ProximityFilter::Ptr filter = _bpfilter->update(state);
	FKTable fk(state);
	ProximityStrategyData data;
	// next we query the BP filter for framepairs that are possibly in collision
	while( !filter->isEmpty() ){
		const FramePair& pair = filter->frontAndPop();

		//std::cout << pair.first->getName() << " " << pair.second->getName() << std::endl;

		// and lastly we use the dispatcher to find the strategy the
		// is required to compute the narrowphase collision
		const ProximityModel::Ptr &a = _frameToModels[*pair.first];
		const ProximityModel::Ptr &b = _frameToModels[*pair.second];

		if(a==NULL || b==NULL)
			continue;

		const Transform3D<> aT = fk.get(*pair.first);
		const Transform3D<> bT = fk.get(*pair.second);
		bool res = _npstrategy->inCollision(a, aT, b, bT, data);
        if( res ){
			if (result) {
				result->collidingFrames.insert(pair);
				if (stopAtFirstContact)
					return true;
			} else
				return true;
        }
	}

	if(result)
	    return result->collidingFrames.size()>0;
    return false;
}

#ifdef RW_USE_DEPRECATED
void CollisionDetector::addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom){
	_bpfilter->addModel(frame, geom);
	// todo: remember to update all midphase filters

	_npstrategy->addModel(frame, geom);
	// now remember to add the proximity model to the framemap
	// todo: make sure to check if the model is allready there
	_frameToModels[*frame] = _npstrategy->getModel(frame);
}

void CollisionDetector::addModel(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geom) {
	_bpfilter->addModel(frame, *geom);
	// todo: remember to update all midphase filters

	_npstrategy->addModel(frame, *geom);
	// now remember to add the proximity model to the framemap
	// todo: make sure to check if the model is allready there
	_frameToModels[*frame] = _npstrategy->getModel(frame);
}

void CollisionDetector::removeModel(rw::kinematics::Frame* frame, const std::string& geoid){
	_bpfilter->removeModel(frame, geoid);
	// todo: remember to update all midphase filters

	// now use the dispatcher to find the right ProximityModel to add the geom to
	if(!_npstrategy->hasModel(frame)){
		RW_THROW("Frame does not have any proximity models attached!");
	}

	ProximityModel::Ptr model = _npstrategy->getModel(frame);
	_npstrategy->removeGeometry(model.get(), geoid);
	_frameToModels[*frame] = _npstrategy->getModel(frame);
}

#endif //RW_USE_DEPRECATED

void CollisionDetector::addGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geometry) {
	if (geometry == NULL) {
		RW_THROW("Unable to add NULL as geometry");
	}
	_npstrategy->addModel(frame, *geometry);
	_frameToModels[*frame] = _npstrategy->getModel(frame); 

	_bpfilter->addGeometry(frame, geometry);

	
}

void CollisionDetector::removeGeometry(rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr geo) {
	removeGeometry(frame, geo->getId());
}

void CollisionDetector::removeGeometry(rw::kinematics::Frame* frame, const std::string geoid) {
	_bpfilter->removeGeometry(frame, geoid);
	// todo: remember to update all midphase filters

	// now use the dispatcher to find the right ProximityModel to add the geom to
	if(!_npstrategy->hasModel(frame)){
		RW_THROW("Frame does not have any proximity models attached!");
	}

	ProximityModel::Ptr model = _npstrategy->getModel(frame);
	_npstrategy->removeGeometry(model.get(), geoid);
	_frameToModels[*frame] = _npstrategy->getModel(frame);
}


void CollisionDetector::addRule(const ProximitySetupRule& rule) {
	_bpfilter->addRule(rule);
}

void CollisionDetector::removeRule(const ProximitySetupRule& rule) {
	_bpfilter->removeRule(rule);
}



std::vector<std::string> CollisionDetector::getGeometryIDs(rw::kinematics::Frame *frame) {
	if(!_frameToModels.has(*frame))
		return std::vector<std::string>();
	ProximityModel::Ptr model = _frameToModels[*frame];
	if(model==NULL)
		return std::vector<std::string>();
	return model->getGeometryIDs();
}


bool CollisionDetector::hasGeometry(rw::kinematics::Frame* frame, const std::string& geometryId) {
	std::vector<std::string> names = getGeometryIDs(frame);
	BOOST_FOREACH(const std::string& name, names) {
		if (name == geometryId)
			return true;
	}
	return false;
}
