/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "LogCollisionResult.hpp"
#include "SimulatorLogScope.hpp"
#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "LogPositions.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwsim::log;

LogCollisionResult::LogCollisionResult(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogCollisionResult::~LogCollisionResult() {
}

void LogCollisionResult::read(class InputArchive& iarchive, const std::string& id) {
	_results.clear();
	unsigned int n;
	n = iarchive.readUInt("Results");
	_results.resize(n);
	BOOST_FOREACH(ResultInfo& info, _results) {
		info.frameA = iarchive.readString("FrameA");
		info.frameB = iarchive.readString("FrameB");
		iarchive.read(info.geoNamesA, "GeometryIDsA");
		iarchive.read(info.geoNamesB, "GeometryIDsB");
		iarchive.read(info.result._aTb,"aTb");
		n = iarchive.readUInt("CollisionPairs");
		info.result._collisionPairs.resize(n);
		BOOST_FOREACH(CollisionStrategy::Result::CollisionPair& pair, info.result._collisionPairs) {
			pair.geoIdxA = iarchive.readInt("GeoIdxA");
			pair.geoIdxB = iarchive.readInt("GeoIdxB");
			pair.startIdx = iarchive.readInt("StartIdx");
			pair.size = iarchive.readInt("Size");
		}
		n = iarchive.readUInt("GeomPrimIds");
		info.result._geomPrimIds.resize(n);
		typedef std::pair<int, int> IntPair;
		BOOST_FOREACH(IntPair& pair, info.result._geomPrimIds) {
			pair.first = iarchive.readInt("First");
			pair.second = iarchive.readInt("Second");
		}
		info.result._nrBVTests = iarchive.readInt("NrBVTests");
		info.result._nrPrimTests = iarchive.readInt("NrPrimTests");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogCollisionResult::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_results.size(),"Results");
	BOOST_FOREACH(const ResultInfo& info, _results) {
		oarchive.write(info.frameA,"FrameA");
		oarchive.write(info.frameB,"FrameB");
		oarchive.write(info.geoNamesA, "GeometryIDsA");
		oarchive.write(info.geoNamesB, "GeometryIDsB");
		oarchive.write(info.result._aTb,"aTb");
		oarchive.write(info.result._collisionPairs.size(),"CollisionPairs");
		BOOST_FOREACH(const CollisionStrategy::Result::CollisionPair& pair, info.result._collisionPairs) {
			oarchive.write(pair.geoIdxA,"GeoIdxA");
			oarchive.write(pair.geoIdxB,"GeoIdxB");
			oarchive.write(pair.startIdx,"StartIdx");
			oarchive.write(pair.size,"Size");
		}
		oarchive.write(info.result._geomPrimIds.size(),"GeomPrimIds");
		typedef std::pair<int, int> IntPair;
		BOOST_FOREACH(const IntPair& pair, info.result._geomPrimIds) {
			oarchive.write(pair.first,"First");
			oarchive.write(pair.second,"Second");
		}
		oarchive.write(info.result._nrBVTests,"NrBVTests");
		oarchive.write(info.result._nrPrimTests,"NrPrimTests");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogCollisionResult::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogCollisionResult::getLinkedEntries() const {
	if (_positions == NULL)
		return std::list<SimulatorLogEntry::Ptr>();
	else
		return std::list<SimulatorLogEntry::Ptr>(1,_positions);
}

bool LogCollisionResult::autoLink() {
	_positions = NULL;
	// Link to last position entry in tree
	SimulatorLogScope* scope = getParent();
	if (scope == NULL)
		return false;
	SimulatorLog* find = this;
	bool found = false;
	while(scope != NULL && !found) {
		// Find position of entry
		const std::vector<SimulatorLog::Ptr> children = scope->getChildren();
		std::vector<SimulatorLog::Ptr>::const_reverse_iterator it;
		for(it = children.rbegin(); it != children.rend(); it++) {
			if (it->isNull())
				continue;
			if (it->get() == find) {
				break;
			}
		}
		if (it != children.rend()) {
			if (it->get() == find)
				it++;
		}
		// Now search backwards
		for(; it != children.rend(); it++) {
			RW_ASSERT(*it != NULL);
			_positions = (*it).cast<LogPositions>();
			if (_positions != NULL) {
				found = true;
				break;
			}
		}
		find = scope;
		scope = scope->getParent();
	}
	return _positions != NULL;
}

SimulatorLogEntry::Ptr LogCollisionResult::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogCollisionResult(parent));
}

std::string LogCollisionResult::getTypeID() {
	return "CollisionResult";
}

const std::vector<LogCollisionResult::ResultInfo>& LogCollisionResult::getResults() const {
	return _results;
}

void LogCollisionResult::addResult(const CollisionStrategy::Result& result) {
	_results.resize(_results.size()+1);
	ResultInfo& info = _results.back();
	info.result = result;
	const ProximityModel::Ptr a = result.a;
	const ProximityModel::Ptr b = result.b;
	if (a != NULL) {
		const Frame* const frame = a->getFrame();
		if (frame != NULL)
			info.frameA = frame->getName();
		info.geoNamesA = a->getGeometryIDs();
		info.result.a = NULL;
	}
	if (b != NULL) {
		const Frame* const frame = b->getFrame();
		if (frame != NULL)
			info.frameB = frame->getName();
		info.geoNamesB = b->getGeometryIDs();
		info.result.b = NULL;
	}
}

void LogCollisionResult::addResults(const std::vector<CollisionStrategy::Result>& results) {
	BOOST_FOREACH(const CollisionStrategy::Result& result, results) {
		addResult(result);
	}
}

LogPositions::Ptr LogCollisionResult::getPositions() const {
	return _positions;
}
