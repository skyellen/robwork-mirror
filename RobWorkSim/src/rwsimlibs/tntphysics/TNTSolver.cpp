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

#include "TNTSolver.hpp"
#include "TNTSolverSVD.hpp"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwsimlibs::tntphysics;

TNTSolver::Factory::Factory():
	ExtensionPoint<TNTSolver>("rwsimlibs.tntphysics.TNTSolver", "TNTSolver extension point.")
{
}

std::vector<std::string> TNTSolver::Factory::getSolvers() {
	std::vector<std::string> solvers;
	solvers.push_back("SVD");
	TNTSolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		solvers.push_back( ext.getProperties().get("solverID",ext.name) );
	}
	return solvers;
}

bool TNTSolver::Factory::hasSolver(const std::string& solverType) {
	if (solverType == "SVD")
		return true;
	TNTSolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("solverID",ext.name) == solverType)
            return true;
	}
	return false;
}

const TNTSolver* TNTSolver::Factory::makeSolver(const std::string& solverType, const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity) {
	if (solverType == "SVD")
		return new TNTSolverSVD(manager,gravity);
	TNTSolver::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("solverID",ext->getName() ) == solverType){
			const rw::common::Ptr<const TNTSolver> base = ext->getObject().cast<const TNTSolver>();
			return base->createSolver(manager,gravity);
		}
	}
	return NULL;
}
