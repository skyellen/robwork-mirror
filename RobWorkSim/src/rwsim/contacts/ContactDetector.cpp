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
		bool matched = false;
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

		std::list<StrategyTable>::const_iterator it;
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
						contacts = stratMatch.strategy->findContacts(modelA.get(), aT, modelB.get(), bT, stratData);
						data.setStrategyData(stratData);
						pairIt = geoPairs.erase(pairIt);
				        if( contacts.size() > 0 ){
							res.insert(res.end(),contacts.begin(),contacts.end());
				        }
					}
				}
				matched = true;
			}
		}
	}

	data.setContacts(res);
	return res;
}

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

	std::list<StrategyTable>::const_iterator it;
	for (it = _strategies.begin(); it != _strategies.end(); it++) {
		std::vector<Cell> row(7, Cell());
		StrategyTable strategy = *it;

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
