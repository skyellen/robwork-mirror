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

#include "RWPELogContactTracking.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include <rwsim/contacts/Contact.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>

using namespace rw::common;
using namespace rwsim::contacts;
using namespace rwsim::log;
using namespace rwsimlibs::rwpe;

RWPELogContactTracking::RWPELogContactTracking(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

RWPELogContactTracking::~RWPELogContactTracking() {
}

void RWPELogContactTracking::read(class InputArchive& iarchive, const std::string& id) {
	{
		const unsigned int n = iarchive.readUInt("Before");
		before.resize(n);
		for (unsigned int i = 0; i < n; i++) {
			before[i].read(iarchive,"");
		}
	}
	{
		const unsigned int n = iarchive.readUInt("After");
		after.resize(n);
		for (unsigned int i = 0; i < n; i++) {
			after[i].read(iarchive,"");
		}
	}
	{
		const unsigned int n = iarchive.readUInt("Added");
		added.resize(n);
		for (unsigned int i = 0; i < n; i++) {
			added[i].read(iarchive,"");
		}
	}
	{
		const unsigned int n = iarchive.readUInt("Gone");
		gone.resize(n);
		for (unsigned int i = 0; i < n; i++) {
			gone[i].read(iarchive,"");
		}
	}
	SimulatorLogEntry::read(iarchive,id);
}

void RWPELogContactTracking::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(before.size(),"Before");
	BOOST_FOREACH(const Contact& c, before) {
		c.write(oarchive,"");
	}
	oarchive.write(after.size(),"After");
	BOOST_FOREACH(const Contact& c, after) {
		c.write(oarchive,"");
	}
	oarchive.write(added.size(),"Added");
	BOOST_FOREACH(const Contact& c, added) {
		c.write(oarchive,"");
	}
	oarchive.write(gone.size(),"Gone");
	BOOST_FOREACH(const Contact& c, gone) {
		c.write(oarchive,"");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string RWPELogContactTracking::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> RWPELogContactTracking::getLinkedEntries() const {
	// Link to last position entry in tree
	/*SimulatorLogScope* scope = getParent();
	while(scope != NULL) {
		std::vector<SimulatorLog::Ptr> children = scope->getChildren();
		std::vector<SimulatorLog::Ptr>::const_reverse_iterator it;
		for(it = children.rbegin(); it != children.rend(); it++) {
			const SimulatorLogPositions::Ptr pos = (*it).cast<SimulatorLogPositions>();
			if (pos != NULL) {
				return std::list<SimulatorLogEntry::Ptr>(1,pos);
			}
		}
		scope = scope->getParent();
	}*/
	return std::list<SimulatorLogEntry::Ptr>();
}

bool RWPELogContactTracking::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr RWPELogContactTracking::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new RWPELogContactTracking(parent));
};

std::string RWPELogContactTracking::getTypeID() {
	return "RWPELogContactTracking";
}
