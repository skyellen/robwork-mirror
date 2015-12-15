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

#include <rwsim/contacts/Contact.hpp>
#include "RWPERollbackMethod.hpp"
#include "RWPERollbackMethodRidder.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rwsim::contacts;
using namespace rwsimlibs::rwpe;

RWPERollbackMethod::RWPERollbackMethod() {
}

RWPERollbackMethod::~RWPERollbackMethod() {
}

RWPERollbackMethod::Sample::Sample():
	time(0)
{
}

RWPERollbackMethod::Sample::Sample(double time, const std::vector<Contact>& contacts):
	time(time)
{
	BOOST_FOREACH(const Contact& c, contacts) {
		if (distance.has(c.getFrameA(),c.getFrameB())) {
			const double dist = distance(c.getFrameA(),c.getFrameB());
			if ((-c.getDepth()) < dist)
				distance(c.getFrameA(),c.getFrameB()) = -c.getDepth();
		} else {
			distance(c.getFrameA(),c.getFrameB()) = -c.getDepth();
			if (c.getFrameA() < c.getFrameB())
				framePairs.insert(std::make_pair<const Frame*,const Frame*>(c.getFrameA(),c.getFrameB()));
			else
				framePairs.insert(std::make_pair<const Frame*,const Frame*>(c.getFrameB(),c.getFrameA()));
		}
	}
}

bool RWPERollbackMethod::SampleCompare::operator()(const Sample& s1, const Sample& s2) const {
	if (s1.time != s2.time)
		return s1.time < s2.time;
	return false;
}

RWPERollbackMethod::RollbackData* RWPERollbackMethod::createData() const {
	return new RollbackData();
}

RWPERollbackMethod::Factory::Factory():
	ExtensionPoint<RWPERollbackMethod>("rwsimlibs.rwpe.RWPERollbackMethod", "RWPERollbackMethod extension point.")
{
}

std::vector<std::string> RWPERollbackMethod::Factory::getMethods() {
    std::vector<std::string> ids;
    RWPERollbackMethod::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    ids.push_back("Secant");
    ids.push_back("Ridder");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("methodID",ext.name) );
    }
    return ids;
}

bool RWPERollbackMethod::Factory::hasMethod(const std::string& method) {
    if( method == "Secant")
        return true;
    if( method == "Ridder")
        return true;
    RWPERollbackMethod::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("methodID",ext.name) == method)
            return true;
    }
    return false;
}

RWPERollbackMethod::Ptr RWPERollbackMethod::Factory::makeMethod(const std::string& method) {
    if( method == "Secant")
        //return new RWPERollbackMethodSecant();
    	return NULL;
    if( method == "Ridder")
        return new RWPERollbackMethodRidder();
    RWPERollbackMethod::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
		if(ext->getProperties().get("methodID",ext->getName() ) == method){
			return ext->getObject().cast<const RWPERollbackMethod>();
		}
	}
	return NULL;
}
