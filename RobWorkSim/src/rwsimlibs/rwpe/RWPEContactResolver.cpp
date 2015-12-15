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

#include <boost/foreach.hpp>
#include "RWPEContactResolverFull.hpp"
#include "RWPEContactResolver.hpp"
#include "RWPEContactResolverNonPenetration.hpp"

using namespace rw::common;
using namespace rwsimlibs::rwpe;

void RWPEContactResolver::addDefaultProperties(PropertyMap& map) const {
}

RWPEContactResolver::Factory::Factory():
	ExtensionPoint<RWPEContactResolver>("rwsimlibs.rwpe.RWPEContactResolver", "RWPEContactResolver extension point.")
{
}

std::vector<std::string> RWPEContactResolver::Factory::getResolvers() {
	std::vector<std::string> resolvers;
	resolvers.push_back("NonPenetration");
	resolvers.push_back("Full");
	RWPEContactResolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		resolvers.push_back( ext.getProperties().get("resolverID",ext.name) );
	}
	return resolvers;
}

bool RWPEContactResolver::Factory::hasResolver(const std::string& resolverType) {
	if (resolverType == "NonPenetration")
		return true;
	else if (resolverType == "Full")
		return true;
	RWPEContactResolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("resolverID",ext.name) == resolverType)
            return true;
	}
	return false;
}

const RWPEContactResolver* RWPEContactResolver::Factory::makeResolver(const std::string& resolverType, const RWPEConstraintSolver* solver) {
	if (resolverType == "NonPenetration")
		return new RWPEContactResolverNonPenetration(solver);
	if (resolverType == "Full")
		return new RWPEContactResolverFull(solver);
	RWPEContactResolver::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("solverID",ext->getName() ) == resolverType){
			const rw::common::Ptr<const RWPEContactResolver> base = ext->getObject().cast<const RWPEContactResolver>();
			return base->createResolver(solver);
		}
	}
	return NULL;
}
