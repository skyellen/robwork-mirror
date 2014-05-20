/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ContactDetector.hpp"
#include "BallBallStrategy.hpp"
#include "ContactStrategyPQP.hpp"
#include "ContactDetectorTracking.hpp"

#include <rw/proximity/BasicFilterStrategy.hpp>

#include <iomanip>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwsim::contacts;

ContactDetector::ContactDetector(WorkCell::Ptr wc, ProximityFilterStrategy::Ptr filter):
	_wc(wc),
	_bpfilter(filter == NULL ? ownedPtr( new BasicFilterStrategy(wc) ) : filter),
	_timer(0)
{
	initializeGeometryMap();
}

ContactDetector::~ContactDetector()
{
	clearStrategies();
}

void ContactDetector::setProximityFilterStrategy(ProximityFilterStrategy::Ptr filter) {
	_bpfilter = filter;
}

double ContactDetector::getTimer() const {
	return _timer;
}

void ContactDetector::setTimer(double value) {
	_timer = value;
}

void ContactDetector::initializeGeometryMap() {
	// run through all objects in workcell and collect the geometric information
	std::vector<Object::Ptr> objects = _wc->getObjects();
	BOOST_FOREACH(Object::Ptr object, objects) {
		BOOST_FOREACH(Geometry::Ptr geom, object->getGeometry() ){
			Frame* frame = geom->getFrame();
			_frameToGeo[*frame].push_back(geom);
		}
	}
}

void ContactDetector::initializeModels(StrategyTableRow &strategy) {
	// run through all objects in workcell and collect the geometric information
	std::vector<Object::Ptr> objects = _wc->getObjects();
	std::vector<Object::Ptr>::iterator itA;
	for (itA = objects.begin(); itA < objects.end(); itA++) {
		std::vector<Object::Ptr>::iterator itB = itA;
		itB++;
		for (; itB < objects.end(); itB++) {
			Object::Ptr oA = *itA;
			Object::Ptr oB = *itB;
			BOOST_FOREACH(Geometry::Ptr geoA, oA->getGeometry() ){
				BOOST_FOREACH(Geometry::Ptr geoB, oB->getGeometry() ){
					if (strategy.strategy->match(geoA->getGeometryData(),geoB->getGeometryData())) {
						std::map<std::string,ContactModel::Ptr>& mapA = strategy.models[*(geoA->getFrame())];
						std::map<std::string,ContactModel::Ptr>& mapB = strategy.models[*(geoB->getFrame())];
						if (mapA.find(geoA->getId()) == mapA.end()) {
							ProximityModel::Ptr model = strategy.strategy->createModel();
							strategy.strategy->addGeometry(model.get(),geoA);
							std::pair<std::string,ContactModel::Ptr> pair = std::pair<std::string,ContactModel::Ptr>(geoA->getId(),model.cast<ContactModel>());
							mapA.insert(pair);
						}
						if (mapB.find(geoB->getId()) == mapB.end()) {
							ProximityModel::Ptr model = strategy.strategy->createModel();
							strategy.strategy->addGeometry(model.get(),geoB);
							std::pair<std::string,ContactModel::Ptr> pair = std::pair<std::string,ContactModel::Ptr>(geoB->getId(),model.cast<ContactModel>());
							mapB.insert(pair);
						}
					}
				}
			}
		}
	}
}

ProximityFilterStrategy::Ptr ContactDetector::getProximityFilterStrategy() const {
	return _bpfilter;
}

std::list<ContactDetector::StrategyTableRow> ContactDetector::getContactStategies() const {
	return _strategies;
}

ContactDetector::StrategyTable ContactDetector::getContactStrategies(const std::string& frameA, const std::string& frameB) const {
	StrategyTable res;
	std::list<StrategyTableRow>::const_iterator it;
	for (it = _strategies.begin(); (it != _strategies.end()); it++) {
		StrategyTableRow stratMatch = *it;
		std::vector<ProximitySetupRule> rules = stratMatch.rules.getProximitySetupRules();
		bool match = false;
		BOOST_FOREACH(ProximitySetupRule &rule, rules) {
			if (rule.match(frameA,frameB))
				match = true;
		}
		if (match) {
			res.push_back(stratMatch);
		}
	}
	return res;
}

ContactDetector::StrategyTable ContactDetector::getContactStrategies(const std::string& frameA, rw::common::Ptr<const Geometry> geometryA, const std::string& frameB, rw::common::Ptr<const Geometry> geometryB) const {
	StrategyTable res;
	StrategyTable table = getContactStrategies(frameA,frameB);
	std::list<StrategyTableRow>::const_iterator it;
	for (it = table.begin(); (it != table.end()); it++) {
		StrategyTableRow stratMatch = *it;
		if (stratMatch.strategy->match(geometryA->getGeometryData(),geometryB->getGeometryData())) {
			res.push_back(stratMatch);
		}
	}
	return res;
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
	std::list<StrategyTableRow>::iterator it = _strategies.begin();
	if (pri > _strategies.size())
		pri = _strategies.size();
	for (std::size_t i = 0; i < pri; i++)
		it++;
	StrategyTableRow matcher;
	matcher.priority = pri;
	matcher.rules = rules;
	matcher.strategy = strategy;
	initializeModels(matcher);
	it = _strategies.insert(it,matcher);
	for (it++; it != _strategies.end(); it++)
		(*it).priority++;
}

void ContactDetector::addContactStrategy(StrategyTableRow &strategy, std::size_t pri) {
	std::list<StrategyTableRow>::iterator it = _strategies.begin();
	if (pri > _strategies.size()-1)
		pri = _strategies.size()-1;
	for (std::size_t i = 0; i < pri; i++)
		it++;
	initializeModels(strategy);
	it = _strategies.insert(it,strategy);
	for (it++; it != _strategies.end(); it++)
		(*it).priority++;
}

void ContactDetector::removeContactStrategy(std::size_t pri) {
	std::list<StrategyTableRow>::iterator it = _strategies.begin();
	if (pri > _strategies.size()-1)
		return;
	it = _strategies.erase(it);
	for (; it != _strategies.end(); it++)
		(*it).priority--;
}

void ContactDetector::clearStrategies() {
	_strategies.clear();
}

void ContactDetector::setContactStrategies(std::list<StrategyTableRow> strategies) {
	clearStrategies();
	_strategies = strategies;
}

void ContactDetector::setDefaultStrategies() {
	PropertyMap emptyMap;
    setDefaultStrategies(emptyMap);
}

void ContactDetector::setDefaultStrategies(const PropertyMap& map) {
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
    	ContactStrategy* strat = new BallBallStrategy();
    	strat->setPropertyMap(map);
    	addContactStrategy(ownedPtr(strat), pri);
    	pri++;
    }
	ContactStrategy* strat = new ContactStrategyPQP();
	strat->setPropertyMap(map);
    addContactStrategy(ownedPtr(strat),pri);
}

std::vector<Contact> ContactDetector::findContacts(const State& state) {
	ContactDetectorData data;
	return findContacts(state,data);
}

std::vector<Contact> ContactDetector::findContacts(const State& state, ContactDetectorData &data) {
	long tstart = (long)TimerUtil::currentTimeUs();

	std::vector<Contact> res;

	ProximityFilter::Ptr filter = _bpfilter->update(state);
	FKTable fk(state);
	while( !filter->isEmpty() ){
		const FramePair& pair = filter->frontAndPop();

		const Transform3D<> aT = fk.get(*pair.first);
		const Transform3D<> bT = fk.get(*pair.second);

		std::vector<Contact> contacts;

		std::vector<Geometry::Ptr> unmatchedA = _frameToGeo[*pair.first];
		std::vector<Geometry::Ptr> unmatchedB = _frameToGeo[*pair.second];
		std::vector<std::pair<Geometry::Ptr,Geometry::Ptr> > geoPairs;
		BOOST_FOREACH(Geometry::Ptr geoA, unmatchedA) {
			BOOST_FOREACH(Geometry::Ptr geoB, unmatchedB) {
				std::pair<Geometry::Ptr,Geometry::Ptr> pair;
				pair.first = geoA;
				pair.second = geoB;
				geoPairs.push_back(pair);
			}
		}

		std::list<StrategyTableRow>::const_iterator it;
		for (it = _strategies.begin(); (it != _strategies.end()); it++) {
			StrategyTableRow stratMatch = *it;
			std::vector<ProximitySetupRule> rules = stratMatch.rules.getProximitySetupRules();
			bool match = false;
			BOOST_FOREACH(ProximitySetupRule &rule, rules) {
				if (rule.match(pair.first->getName(),pair.second->getName()))
					match = true;
			}
			if (match) {
				std::vector<std::pair<Geometry::Ptr,Geometry::Ptr> >::iterator pairIt;
				for (pairIt = geoPairs.begin(); pairIt < geoPairs.end(); pairIt++) {
					Geometry::Ptr geoA = (*pairIt).first;
					Geometry::Ptr geoB = (*pairIt).second;
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
						ContactStrategyData& stratData = data.getStrategyData(modelA.get(),modelB.get());
						contacts = stratMatch.strategy->findContacts(modelA.get(), aT, modelB.get(), bT, stratData);
						pairIt = geoPairs.erase(pairIt);
						pairIt--;
				        if( contacts.size() > 0 ){
							res.insert(res.end(),contacts.begin(),contacts.end());
				        }
					}
				}
			}
		}
	}
	long tend = (long)TimerUtil::currentTimeUs();

	double used = ((double)(tend-tstart))/1000000.;

	_timer += used;
	return res;
}

std::vector<Contact> ContactDetector::findContacts(const State& state, ContactDetectorData &data, ContactDetectorTracking& tracking) {
	long tstart = (long)TimerUtil::currentTimeUs();

	std::vector<Contact> res;
	std::vector<ContactDetectorTracking::ContactInfo>& trackInfo = tracking.getInfo();
	trackInfo.clear();

	ProximityFilter::Ptr filter = _bpfilter->update(state);
	const FKTable fk(state);
	while( !filter->isEmpty() ){
		const FramePair& pair = filter->frontAndPop();

		const Transform3D<> aT = fk.get(*pair.first);
		const Transform3D<> bT = fk.get(*pair.second);

		std::vector<Contact> contacts;

		std::vector<Geometry::Ptr> unmatchedA = _frameToGeo[*pair.first];
		std::vector<Geometry::Ptr> unmatchedB = _frameToGeo[*pair.second];
		std::vector<std::pair<Geometry::Ptr,Geometry::Ptr> > geoPairs;
		BOOST_FOREACH(Geometry::Ptr geoA, unmatchedA) {
			BOOST_FOREACH(Geometry::Ptr geoB, unmatchedB) {
				std::pair<Geometry::Ptr,Geometry::Ptr> pair;
				pair.first = geoA;
				pair.second = geoB;
				geoPairs.push_back(pair);
			}
		}

		std::list<StrategyTableRow>::const_iterator it;
		for (it = _strategies.begin(); (it != _strategies.end()); it++) {
			StrategyTableRow stratMatch = *it;
			std::vector<ProximitySetupRule> rules = stratMatch.rules.getProximitySetupRules();
			bool match = false;
			BOOST_FOREACH(ProximitySetupRule &rule, rules) {
				if (rule.match(pair.first->getName(),pair.second->getName()))
					match = true;
			}
			if (match) {
				std::vector<std::pair<Geometry::Ptr,Geometry::Ptr> >::iterator pairIt;
				for (pairIt = geoPairs.begin(); pairIt < geoPairs.end(); pairIt++) {
					Geometry::Ptr geoA = (*pairIt).first;
					Geometry::Ptr geoB = (*pairIt).second;
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
						ContactStrategyData& stratData = data.getStrategyData(modelA.get(),modelB.get());
						ContactStrategyTracking& stratTracking = tracking.getStrategyTracking(modelA.get(),modelB.get());
						contacts = stratMatch.strategy->findContacts(modelA.get(), aT, modelB.get(), bT, stratData, stratTracking);
						pairIt = geoPairs.erase(pairIt);
						pairIt--;
						if( contacts.size() > 0 ){
							ContactDetectorTracking::ContactInfo info;
							info.frames = pair;
							info.models = std::make_pair<ContactModel*,ContactModel*>(modelA.get(),modelB.get());
							info.strategy = stratMatch.strategy;
							info.tracking = &stratTracking;
							info.total = contacts.size();
							for (std::size_t i = 0; i < contacts.size(); i++) {
								info.id = i;
								trackInfo.push_back(info);
							}
							res.insert(res.end(),contacts.begin(),contacts.end());
						}
					}
				}
			}
		}
	}
	long tend = (long)TimerUtil::currentTimeUs();

	double used = ((double)(tend-tstart))/1000000.;

	_timer += used;

	return res;
}

std::vector<Contact> ContactDetector::updateContacts(const State& state, ContactDetectorData &data, ContactDetectorTracking& tracking) {
	std::vector<Contact> res;
	const FKTable fk(state);
	std::vector<ContactDetectorTracking::ContactInfo>& infos = tracking.getInfo();
	std::vector<ContactDetectorTracking::ContactInfo>::iterator it;
	for (it = infos.begin(); it != infos.end(); it++) {
		ContactDetectorTracking::ContactInfo& info = *it;
		if (info.id > 0)
			continue;
		const Frame* const frameA = info.frames.first;
		const Frame* const frameB = info.frames.second;
		ContactModel* const modelA = info.models.first;
		ContactModel* const modelB = info.models.second;
		const Transform3D<> aT = fk.get(frameA);
		const Transform3D<> bT = fk.get(frameB);
		ContactStrategyData& stratData = data.getStrategyData(modelA,modelB);
		if (!stratData.isInitialized())
			RW_THROW("ContactDetector (updateContacts): could not find initialized ContactStrategyData for the two models in ContactDetectorData!");
		const std::vector<Contact> contacts = info.strategy->updateContacts(modelA, aT, modelB, bT, stratData, *info.tracking);
		const std::size_t total = info.total;
		if (contacts.size() < total) {
			const std::size_t remove = total-contacts.size();
			(*it).total = contacts.size();
			for (std::size_t i = 0; i < total-1; i++) {
				it++;
				(*it).total = contacts.size();
			}
			for (std::size_t i = 0; i < remove; i++) {
				it = infos.erase(it);
				it--;
			}
			//for (std::size_t i = 0; i < contacts.size()-1; i++)
			//	it--; // Skipped in beginning of for loop anyway!
		} else if (contacts.size() > total) {
			const std::size_t add = contacts.size()-total;
			(*it).total = contacts.size();
			for (std::size_t i = 0; i < total-1; i++) {
				it++;
				(*it).total = contacts.size();
			}
			for (std::size_t i = 0; i < add; i++) {
				ContactDetectorTracking::ContactInfo newInfo = *it;
				newInfo.id = total+i;
				newInfo.total = contacts.size();
				it = infos.insert(it,newInfo);
			}
			//for (std::size_t i = 0; i < contacts.size()-1; i++)
			//	it--; // Skipped in beginning of for loop anyway!
		}
		res.insert(res.end(),contacts.begin(),contacts.end());
	}
	return res;
}

struct ContactDetector::Cell {
	enum ALIGNMENT {
		LEFT, RIGHT
	};
	Cell() :
		alignment(LEFT) {
	}
	Cell(std::string string) :
		alignment(LEFT) {
		strings.push_back(string);
	}
	std::vector<std::string> strings;
	ALIGNMENT alignment;
};

void ContactDetector::printStrategyTable() const {
	if (_strategies.size() == 0)
			std::cout << "No strategies registered." << std::endl;
	std::vector<std::vector<Cell> > table;

	std::vector<Cell> header;
	header.push_back(Cell("Priority"));
	header.push_back(Cell("Pattern A"));
	header.push_back(Cell("Pattern B"));
	header.push_back(Cell("Strategy"));
	header.push_back(Cell("Model Frame"));
	header.push_back(Cell("Model Geo"));
	header.push_back(Cell("Model Type"));
	table.push_back(header);

	std::list<StrategyTableRow>::const_iterator it;
	for (it = _strategies.begin(); it != _strategies.end(); it++) {
		std::vector<Cell> row(7, Cell());
		const StrategyTableRow &strategy = *it;

		std::stringstream str;
		str << strategy.priority;
		row[0].strings.push_back(str.str());
		row[0].alignment = Cell::RIGHT;

		std::vector<ProximitySetupRule> rules = strategy.rules.getProximitySetupRules();
		BOOST_FOREACH(ProximitySetupRule rule,rules) {
			row[1].strings.push_back(rule.getPatterns().first);
			row[2].strings.push_back(rule.getPatterns().second);
		}

		row[3].strings.push_back(strategy.strategy->getName());

		std::vector<Frame*> frames = _wc->getFrames();
		BOOST_FOREACH(Frame* frame, frames) {
			bool first = true;
			std::map<std::string, ContactModel::Ptr> models = strategy.models[*frame];
			if (models.size() > 0) {
				std::map<std::string, ContactModel::Ptr>::iterator modelIt;
				for (modelIt = models.begin(); modelIt != models.end(); modelIt++) {
					if (first)
						row[4].strings.push_back(frame->getName());
					else
						row[4].strings.push_back("");
					row[5].strings.push_back(modelIt->first);
					row[6].strings.push_back(modelIt->second->getName());
					first = false;
				}
			}
		}
		table.push_back(row);
	}

	printTable(table, true);
}

void ContactDetector::printTable(const std::vector<std::vector<Cell> > &table, bool header) {
	if (table.size() == 0)
		return;
	if (table[0].size() == 0)
		return;
	std::vector<std::size_t> width;
	BOOST_FOREACH(Cell col, table[0]) {
		width.push_back(0);
	}
	BOOST_FOREACH(const std::vector<Cell> &row, table) {
		std::size_t colI = 0;
		BOOST_FOREACH(Cell col, row) {
			BOOST_FOREACH(std::string str, col.strings) {
				if (width[colI] < str.length())
					width[colI] = str.length();
			}
			colI++;
		}
	}
	std::size_t totalwidth = 0;
	BOOST_FOREACH(std::size_t w, width) {
		totalwidth += w;
	}
	totalwidth += width.size()+1;
	for (std::size_t i = 0; i < totalwidth; i++)
		std::cout << "-";
	std::cout << std::endl;
	BOOST_FOREACH(const std::vector<Cell> &row, table) {
		std::size_t stringI = 0;
		bool more = true;
		while (more) {
			more = false;
			std::cout << "|";
			std::size_t colI = 0;
			BOOST_FOREACH(Cell col, row) {
				if (stringI+1 < col.strings.size())
					more = true;
				if (col.alignment == Cell::RIGHT)
					std::cout << std::right;
				else
					std::cout << std::left;
				if (stringI < col.strings.size())
					std::cout << std::setw(width[colI]) << col.strings[stringI] << "|";
				else
					std::cout << std::setw(width[colI]) << "" << "|";
				colI++;
			}
			stringI++;
			std::cout << std::endl;
		}
		for (std::size_t i = 0; i < totalwidth; i++)
			std::cout << "-";
		std::cout << std::endl;
	}
}

ContactDetector::Ptr ContactDetector::makeDefault(WorkCell::Ptr workcell) {
	ContactDetector::Ptr def = ownedPtr(new ContactDetector(workcell));
	def->setDefaultStrategies();
	return def;
}
