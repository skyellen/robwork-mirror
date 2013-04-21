/*
 * ContactDetector.cpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#include "ContactDetector.hpp"
#include "BallBallStrategy.hpp"

#include <rw/proximity/BasicFilterStrategy.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwsim::contacts;

ContactDetector::ContactDetector(WorkCell::Ptr workcell):
	_wc(workcell),
	_bpfilter(ownedPtr( new BasicFilterStrategy(workcell) ))
{
}

ContactDetector::ContactDetector(WorkCell::Ptr wc, ProximityFilterStrategy::Ptr filter):
	_wc(wc),
	_bpfilter(filter)
{
}

ContactDetector::~ContactDetector()
{
	clearStrategies();
}

void ContactDetector::initializeMap() {
	// run through all objects in workcell and collect the geometric information
	std::vector<Object::Ptr> objects = _wc->getObjects();
	BOOST_FOREACH(Object::Ptr object, objects) {
		BOOST_FOREACH(Geometry::Ptr geom, object->getGeometry() ){
			Frame* frame = geom->getFrame();
			_frameToGeo[*frame].push_back(geom);
		}
	}
}

ProximityFilterStrategy::Ptr ContactDetector::getProximityFilterStrategy() const {
	return _bpfilter;
}

std::list<ContactDetector::StrategyTable> ContactDetector::getContactStategies() const {
	return _strategies;
}

void ContactDetector::addContactStrategy(ContactStrategy::Ptr strategy, std::size_t pri) {
	ProximitySetup rules;
	rules.addProximitySetupRule(ProximitySetupRule::makeInclude("*","*"));
	addContactStrategy(rules, strategy, pri);
}

void ContactDetector::addContactStrategy(ProximitySetupRule rule, ContactStrategy::Ptr strategy, std::size_t pri) {
	ProximitySetup setup;
	setup.addProximitySetupRule(rule);
	addContactStrategy(setup, strategy, pri);
}

void ContactDetector::addContactStrategy(ProximitySetup rules, ContactStrategy::Ptr strategy, std::size_t pri) {
	std::list<StrategyTable>::iterator it = _strategies.begin();
	if (pri > _strategies.size()-1)
		pri = _strategies.size()-1;
	for (std::size_t i = 0; i < pri; i++)
		it++;
	StrategyTable matcher;
	matcher.priority = pri;
	matcher.rules = rules;
	matcher.strategy = strategy;
	it = _strategies.insert(it,matcher);
	for (it++; it != _strategies.end(); it++)
		(*it).priority++;
}

void ContactDetector::addContactStrategy(StrategyTable strategy, std::size_t pri) {
	std::list<StrategyTable>::iterator it = _strategies.begin();
	if (pri > _strategies.size()-1)
		pri = _strategies.size()-1;
	for (std::size_t i = 0; i < pri; i++)
		it++;
	it = _strategies.insert(it,strategy);
	for (it++; it != _strategies.end(); it++)
		(*it).priority++;
}

void ContactDetector::removeContactStrategy(std::size_t pri) {
	std::list<StrategyTable>::iterator it = _strategies.begin();
	if (pri > _strategies.size()-1)
		return;
	it = _strategies.erase(it);
	for (; it != _strategies.end(); it++)
		(*it).priority--;
}

void ContactDetector::clearStrategies() {
	_strategies.clear();
}

void ContactDetector::setContactStrategies(std::list<StrategyTable> strategies) {
	clearStrategies();
	_strategies = strategies;
}

void ContactDetector::setDefaultStrategies() {
    std::vector<Object::Ptr> objects = _wc->getObjects();
    std::size_t spheres = 0;
    BOOST_FOREACH(Object::Ptr object, objects) {
        BOOST_FOREACH(Geometry::Ptr geom, object->getGeometry()){
        	GeometryData::Ptr gdata = geom->getGeometryData();
        	GeometryData::GeometryType gtype = gdata->getType();
            if (gtype == GeometryData::SpherePrim) spheres++;
        }
    }
    std::size_t pri = 0;
    if (spheres > 1) {
    	addContactStrategy(ownedPtr(new BallBallStrategy()), pri);
    	pri++;
    }
}

std::vector<Contact> ContactDetector::findContacts(const State& state) const {
	ContactDetectorData data;
	return findContacts(state,data);
}

std::vector<Contact> ContactDetector::findContacts(const State& state, ContactDetectorData &data) const {
	std::vector<Contact> res;

	ProximityFilter::Ptr filter = _bpfilter->update(state);
	FKTable fk(state);
	while( !filter->isEmpty() ){
		const FramePair& pair = filter->frontAndPop();

		const Transform3D<> aT = fk.get(*pair.first);
		const Transform3D<> bT = fk.get(*pair.second);

		ContactStrategyData stratData(data.getStrategyData());
		std::list<StrategyTable>::const_iterator it;
		bool matched = false;
		std::vector<Contact> contacts;
		for (it = _strategies.begin(); (it != _strategies.end()) && !matched; it++) {
			StrategyTable stratMatch = *it;
			BasicFilterStrategy filterStrat(_wc,stratMatch.rules);
			ProximityFilter::Ptr filterTest = filterStrat.update(state);
			bool match = false;
			while (!filterTest->isEmpty()) {
				const FramePair& frames = filterTest->frontAndPop();
				if (frames.first == pair.first && frames.second == pair.second)
					match = true;
				if (frames.second == pair.first && frames.first == pair.second)
					match = true;
			}
			if (match) {
				// Problem.. matching: what if only some geometries match?
				BOOST_FOREACH(Geometry::Ptr geoA, _frameToGeo[*pair.first]) {
					BOOST_FOREACH(Geometry::Ptr geoB, _frameToGeo[*pair.second]) {
						if (stratMatch.strategy->match(geoA->getGeometryData(),geoB->getGeometryData())) {
							std::map<std::string, ContactModel::Ptr> &mapA = stratMatch.models[*pair.first];
							std::map<std::string, ContactModel::Ptr> &mapB = stratMatch.models[*pair.second];
							if (mapA.find(geoA->getId())==mapA.end()) {
								ProximityModel::Ptr model = stratMatch.strategy->createModel();
								stratMatch.strategy->addGeometry(model.get(),geoA);
							}
							if (mapB.find(geoB->getId())==mapB.end()) {
								ProximityModel::Ptr model = stratMatch.strategy->createModel();
								stratMatch.strategy->addGeometry(model.get(),geoB);
							}
							ContactModel::Ptr modelA = mapA[geoA->getId()];
							ContactModel::Ptr modelB = mapB[geoB->getId()];
							contacts = stratMatch.strategy->findContacts(modelA.get(), aT, modelB.get(), bT, stratData);
							data.setStrategyData(stratData);
						}
					}
				}
				matched = true;
			}
		}
        if( contacts.size() > 0 ){
			res.insert(res.end(),contacts.begin(),contacts.end());
        }
	}

	data.setContacts(res);
	return res;
}
