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

#include "TNTCollisionSolver.hpp"
#include "TNTCollisionSolverSingle.hpp"
#include "TNTCollisionSolverChain.hpp"
#include "TNTCollisionSolverSimultaneous.hpp"
#include "TNTCollisionSolverHybrid.hpp"
#include "TNTRigidBody.hpp"

#include <rwsim/dynamics/RigidBody.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

TNTCollisionSolver::TNTCollisionSolver()
{
};

TNTCollisionSolver::~TNTCollisionSolver() {
};

void TNTCollisionSolver::applyImpulse(const Wrench6D<>& impulse, const Vector3D<>& position, const TNTRigidBody& body, TNTIslandState& tntstate) {
	const VelocityScrew6D<> velW = body.getVelocityW(tntstate);
	const Vector3D<> velWang = velW.angular().axis()*velW.angular().angle();
	const Transform3D<>& wTcom = body.getWorldTcom(tntstate);
	const InertiaMatrix<>& inertiaInv = wTcom.R()*body.getRigidBody()->getBodyInertiaInv()*inverse(wTcom.R());

	const Vector3D<> linVel = velW.linear() + body.getRigidBody()->getMassInv()*impulse.force();
	const Vector3D<> angVel = velWang + inertiaInv*(impulse.torque()+cross(position-wTcom.P(),impulse.force()));

	body.setVelocityW(VelocityScrew6D<>(linVel,EAA<>(angVel)),tntstate);
}

void TNTCollisionSolver::applyImpulses(const std::vector<Wrench6D<> >& impulses, const std::vector<Vector3D<> >& positions, const TNTRigidBody& body, TNTIslandState& tntstate) {
	if (impulses.size() != positions.size())
		RW_THROW("TNTCollisionSolver (applyImpulses): the number of impulses does not match the number of positions!");

	const VelocityScrew6D<> velW = body.getVelocityW(tntstate);
	const Vector3D<> velWang = velW.angular().axis()*velW.angular().angle();
	const Transform3D<>& wTcom = body.getWorldTcom(tntstate);
	const InertiaMatrix<>& inertiaInv = wTcom.R()*body.getRigidBody()->getBodyInertiaInv()*inverse(wTcom.R());
	const double massInv = body.getRigidBody()->getMassInv();

	Vector3D<> linVel = velW.linear();
	Vector3D<> angVel = velWang;
	std::size_t i = 0;
	BOOST_FOREACH(const Wrench6D<>& impulse,impulses) {
		linVel += massInv*impulse.force();
		angVel += inertiaInv*(impulse.torque()+cross(positions[i]-wTcom.P(),impulse.force()));
		i++;
	}

	body.setVelocityW(VelocityScrew6D<>(linVel,EAA<>(angVel)),tntstate);
}

void TNTCollisionSolver::addDefaultProperties(PropertyMap& map) const {
}

PropertyMap TNTCollisionSolver::getDefaultMap() const {
	PropertyMap map;
	addDefaultProperties(map);
	return map;
}

class TNTCollisionSolver::TNTCollisionSolverNone: public TNTCollisionSolver {
public:
	TNTCollisionSolverNone() {};
	virtual ~TNTCollisionSolverNone() {};
	virtual void doCollisions(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			rw::common::Ptr<ThreadTask> task) const
	{
		if (contacts.size() > 0)
			RW_THROW("TNTCollisionSolver::TNTCollisionSolverNone (applyImpulses): this collision solver can not handle any collisions (use this solver only if it is known that there will be no collisions).");
	}
};

TNTCollisionSolver::Factory::Factory():
	ExtensionPoint<TNTCollisionSolver>("rwsimlibs.tntphysics.TNTCollisionSolver", "TNTCollisionSolver extension point.")
{
}

std::vector<std::string> TNTCollisionSolver::Factory::getSolvers() {
    std::vector<std::string> ids;
    TNTCollisionSolver::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    ids.push_back("None");
    ids.push_back("Single");
    ids.push_back("Chain");
    ids.push_back("Simultaneous");
    ids.push_back("Hybrid");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("solverID",ext.name) );
    }
    return ids;
}

bool TNTCollisionSolver::Factory::hasSolver(const std::string& method) {
    if( method == "None")
        return true;
    else if( method == "Single")
        return true;
    else if( method == "Chain")
        return true;
    else if( method == "Simultaneous")
        return true;
    else if( method == "Hybrid")
        return true;
    TNTCollisionSolver::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("solverID",ext.name) == method)
            return true;
    }
    return false;
}

TNTCollisionSolver::Ptr TNTCollisionSolver::Factory::makeSolver(const std::string& method) {
    if( method == "None")
        return ownedPtr(new TNTCollisionSolverNone());
    else if( method == "Single")
        return ownedPtr(new TNTCollisionSolverSingle());
    else if( method == "Chain")
        return ownedPtr(new TNTCollisionSolverChain());
    else if( method == "Simultaneous")
        return ownedPtr(new TNTCollisionSolverSimultaneous());
    else if( method == "Hybrid")
        return ownedPtr(new TNTCollisionSolverHybrid());
    TNTCollisionSolver::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
		if(ext->getProperties().get("solverID",ext->getName() ) == method){
			return ext->getObject().cast<const TNTCollisionSolver>();
		}
	}
	return NULL;
}
