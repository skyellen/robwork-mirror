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

#include "TNTContactResolver.hpp"
#include "TNTContactResolverNonPenetration.hpp"
#include "TNTContactResolverHeuristic.hpp"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwsimlibs::tntphysics;

TNTContactResolver::Factory::Factory():
	ExtensionPoint<TNTContactResolver>("rwsimlibs.tntphysics.TNTContactResolver", "TNTContactResolver extension point.")
{
}

std::vector<std::string> TNTContactResolver::Factory::getResolvers() {
	std::vector<std::string> resolvers;
	resolvers.push_back("NonPenetration");
	resolvers.push_back("Heuristic");
	TNTContactResolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		resolvers.push_back( ext.getProperties().get("resolverID",ext.name) );
	}
	return resolvers;
}

bool TNTContactResolver::Factory::hasResolver(const std::string& resolverType) {
	if (resolverType == "Heuristic")
		return true;
	if (resolverType == "NonPenetration")
		return true;
	TNTContactResolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("resolverID",ext.name) == resolverType)
            return true;
	}
	return false;
}

const TNTContactResolver* TNTContactResolver::Factory::makeResolver(const std::string& resolverType, const TNTSolver* solver) {
	if (resolverType == "Heuristic")
		return new TNTContactResolverHeuristic(solver);
	if (resolverType == "NonPenetration")
		return new TNTContactResolverNonPenetration(solver);
	TNTContactResolver::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("solverID",ext->getName() ) == resolverType){
			const rw::common::Ptr<const TNTContactResolver> base = ext->getObject().cast<const TNTContactResolver>();
			return base->createResolver(solver);
		}
	}
	return NULL;
}
