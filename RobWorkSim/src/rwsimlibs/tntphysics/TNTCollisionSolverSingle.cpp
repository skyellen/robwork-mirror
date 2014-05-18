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

#include "TNTCollisionSolverSingle.hpp"

#include "TNTSettings.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTContact.hpp"
#include "TNTIslandState.hpp"
#include "TNTMaterialMap.hpp"
#include "TNTRigidBody.hpp"

#include "TNTRestitutionModel.hpp"

#include <rwsim/dynamics/RigidBody.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTCollisionSolverSingle::TNTCollisionSolverSingle():
	_checkNoChain(true)
{
}

TNTCollisionSolverSingle::~TNTCollisionSolverSingle() {
}

void TNTCollisionSolverSingle::applyImpulses(
		const std::vector<const TNTContact*>& contacts,
		const TNTBodyConstraintManager& bc,
		const TNTMaterialMap* map,
		TNTIslandState& tntstate,
		const State& rwstate) const
{
	// Arrange contacts in bins (if there are multiple collisions between independent pairs of objects)
	std::vector<std::vector<const TNTContact*> > bins;
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		bool found = false;
		const TNTBody* bodyA = contact->getParent();
		const TNTBody* bodyB = contact->getChild();
		if (bodyA > bodyB) {
			bodyA = contact->getChild();
			bodyB = contact->getParent();
		}
		// Search for existing bin for the object pair.
		BOOST_FOREACH(std::vector<const TNTContact*>& bin, bins) {
			const TNTBody* const parent = bin.front()->getParent();
			const TNTBody* const child = bin.front()->getChild();
			if ((bodyA == parent && bodyB == child) || (bodyA == child && bodyB == parent)) {
				bin.push_back(contact);
				found = true;
				break;
			}
		}
		if (!found) {
			// Create new bin for the contact
			const TNTRigidBody* const rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
			const TNTRigidBody* const rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
			if (!rBodyA && !rBodyB) {
				TNT_DEBUG_BOUNCING("The collision solver can not solve impulses between the two kinematic/static bodies \"" << bodyA->get()->getName() << "\" and \"" << bodyB->get()->getName() << "\".");
				RW_THROW("TNTCollisionSolverSingle (applyImpulses): cannot solve impulses for contact between the two kinematic/static bodies \"" << bodyA->get()->getName() << "\" and \"" << bodyB->get()->getName() << "\".");
			}
			if (_checkNoChain) {
				bool chain = false;
				const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyA, tntstate);
				if (rBodyA) {
					const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyA, tntstate);
					BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
						const TNTBody* cBodyA = constraint->getParent();
						const TNTBody* cBodyB = constraint->getChild();
						if ((bodyA != cBodyA || bodyB != cBodyB) && (bodyA != cBodyB || bodyB != cBodyA)) {
							chain = true;
						}
					}
				}
				if (!chain && bodyB)
					const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyB, tntstate);
					BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
						const TNTBody* cBodyA = constraint->getParent();
						const TNTBody* cBodyB = constraint->getChild();
						if ((bodyA != cBodyA || bodyB != cBodyB) && (bodyA != cBodyB || bodyB != cBodyA)) {
							chain = true;
						}
					}
				if (chain) {
					TNT_DEBUG_BOUNCING("Chain encountered in the collision solver for single collisions.");
					RW_THROW("TNTCollisionSolverSingle (applyImpulses): this collision solver can not handle impulse chains!");
				}
			}
			bins.push_back(std::vector<const TNTContact*>(1,contact));
		}
	}
	// Now handle each independent bin (could be done in multiple threads?)
	BOOST_FOREACH(const std::vector<const TNTContact*>& bin, bins) {
		RW_ASSERT(bin.size() > 0);
		const TNTBody* const bodyA = bin[0]->getParent();
		const TNTBody* const bodyB = bin[0]->getChild();
		const std::list<const TNTConstraint*> constraints = bc.getConstraints(bodyA,bodyB,tntstate);
		const TNTRestitutionModel &model = map->getRestitutionModel(*bodyA,*bodyB);
		resolveContacts(bodyA, bodyB, bin, constraints, model, bc, tntstate, rwstate);
	}
}

void TNTCollisionSolverSingle::applyImpulses(
		const TNTBody* bodyA,
		const TNTBody* bodyB,
		const TNTBodyConstraintManager& bc,
		const TNTMaterialMap* map,
		TNTIslandState& tntstate,
		const State& rwstate) const
{
	const std::vector<const TNTContact*> empty;
	const std::list<const TNTConstraint*> constraints = bc.getConstraints(bodyA,bodyB,tntstate);
	const TNTRestitutionModel &model = map->getRestitutionModel(*bodyA,*bodyB);
	resolveContacts(bodyA, bodyB, empty, constraints, model, bc, tntstate, rwstate);
}

bool TNTCollisionSolverSingle::doCheckNoChain() const {
	return _checkNoChain;
}

void TNTCollisionSolverSingle::setCheckNoChain(bool enable) {
	_checkNoChain = enable;
}

void TNTCollisionSolverSingle::resolveContacts(
		const TNTBody* parent,
		const TNTBody* child,
		const std::vector<const TNTContact*>& contacts,
		const std::list<const TNTConstraint*>& constraints,
		const TNTRestitutionModel& restitutionModel,
		const TNTBodyConstraintManager& bc,
		TNTIslandState& tntstate,
		const State& rwstate) const
{
	TNTIslandState tmpState;
	// Construct initial lists assuming all known contacts will be leaving and only the new contacts will penetrate.
	std::vector<const TNTContact*> penetratingContacts = contacts;
	std::vector<const TNTContact*> leavingContacts;
	std::vector<const TNTConstraint*> nonContacts;
	BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
		const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
		if (contact) {
			bool found = false;
			BOOST_FOREACH(const TNTContact* const penContact, penetratingContacts) {
				if (penContact == contact) {
					found = true;
					break;
				}
			}
			if (!found) {
				leavingContacts.push_back(contact);
			}
		} else {
			nonContacts.push_back(contact);
		}
	}
	// Find solution repeatedly and add penetrating contacts until no contacts are penetrating.
	// Also remove non-penetrating contacts! (otherwise solver will throw exception!)
	bool noPenetrations = false;
	while (!noPenetrations) {
		tmpState = tntstate;
		if (penetratingContacts.size() > 0 || nonContacts.size() > 0) {
			solve(parent, child, penetratingContacts, nonContacts, restitutionModel, tmpState, rwstate);
		}
#if TNT_DEBUG_ENABLE_BOUNCING
		static const double TOLERANCE = 1e-4;
		if (leavingContacts.size() == 0) {
			BOOST_FOREACH(const TNTContact* const contact, penetratingContacts) {
				const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,rwstate).linear();
				const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,rwstate).linear();
				const Vector3D<> linRelVel = linVelI-linVelJ;
				const Vector3D<> nij = contact->getNormalW(tmpState);
				const double vel = dot(-linRelVel,nij);
				const bool leaving = vel >= 0;
				if (!leaving) {
					if (vel < -TOLERANCE) {
						RW_THROW("TNTCollisionSolverSingle (resolveContacts): a contact did not become non-penetrating when applying impulse. Velocity is " << vel << " and should be bigger than " << -TOLERANCE);
					}
				}
			}
			BOOST_FOREACH(const TNTConstraint* const constraint, nonContacts) {
				if (constraint->getDimVelocity() == 0)
					continue;
				const VelocityScrew6D<> velI = constraint->getVelocityParentW(tmpState,rwstate);
				const VelocityScrew6D<> velJ = constraint->getVelocityChildW(tmpState,rwstate);
				const Vector3D<> angVelI = velI.angular().angle()*velI.angular().axis();
				const Vector3D<> angVelJ = velJ.angular().angle()*velJ.angular().axis();
				const Vector3D<> linRelVel = velI.linear()-velJ.linear();
				const Vector3D<> angRelVel = angVelI-angVelJ;
				const Rotation3D<> linRot = constraint->getLinearRotationParentW(tmpState);
				const Rotation3D<> angRot = constraint->getLinearRotationParentW(tmpState);
				const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
				Vector3D<> lin = Vector3D<>::zero();
				Vector3D<> ang = Vector3D<>::zero();
				for (std::size_t i = 0; i < 3; i++) {
					if (modes[i] == TNTConstraint::Velocity) {
						lin += dot(linRelVel,linRot.getCol(i))*linRot.getCol(i);
					}
				}
				for (std::size_t i = 3; i < 6; i++) {
					if (modes[i] == TNTConstraint::Velocity) {
						ang += dot(angRelVel,angRot.getCol(i-3))*angRot.getCol(i-3);
					}
				}
				if (lin.norm2() > TOLERANCE) {
					RW_THROW("TNTCollisionSolverSingle (resolveContacts): a constraint violated the desired linear velocity when applying impulse.");
				}
				if (ang.norm2() > TOLERANCE) {
					RW_THROW("TNTCollisionSolverSingle (resolveContacts): a constraint violated the desired angular velocity when applying impulse.");
				}
			}
		}
#endif
		noPenetrations = true;
		std::vector<const TNTContact*>::iterator it;
		for (it = leavingContacts.begin(); it != leavingContacts.end(); it++) {
			const TNTContact* const contact = *it;
			/*bool found = false;
			BOOST_FOREACH(const TNTContact* const inputContact, contacts) {
				if (inputContact == contact) {
					found = true;
					break;
				}
			}
			if (found) {
				continue;
			}*/
			const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,rwstate).linear();
			const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,rwstate).linear();
			const Vector3D<> linRelVel = linVelI-linVelJ;
			const Vector3D<> nij = contact->getNormalW(tmpState);
			const bool leaving = dot(-linRelVel,nij) >= 0;
			if (!leaving) {
				penetratingContacts.push_back(contact);
				it = leavingContacts.erase(it);
				it--;
				noPenetrations = false;
			}
		}
	}
	/*if (leavingContacts.size() > 0) {
		TNT_DEBUG_BOUNCING(leavingContacts.size() << " contacts resolved as leaving.");
		BOOST_FOREACH(const TNTContact* const contact, leavingContacts) {
			bc.removeTemporaryConstraint(contact,tmpState);
		}
	}
	std::size_t removed = 0;
	BOOST_FOREACH(const TNTContact* const contact, penetratingContacts) {
		const TNTRestitutionModel::Values restitution = restitutionModel.getRestitution(*contact,tntstate,rwstate);
		if (restitution.normal > 0) {
			bc.removeTemporaryConstraint(contact,tmpState);
			removed++;
		}
	}
	if (removed > 0)
		TNT_DEBUG_BOUNCING("Removing " << removed << " contacts with normal restitution.");
		*/
	tntstate = tmpState;
}

void TNTCollisionSolverSingle::solve(
		const TNTBody* const parent,
		const TNTBody* const child,
		const std::vector<const TNTContact*>& contacts,
		const std::vector<const TNTConstraint*>& constraints,
		const TNTRestitutionModel& restitutionModel,
		TNTIslandState& tntstate,
		const State& rwstate) const
{
	RW_ASSERT(contacts.size() > 0 || constraints.size() > 0);
	const TNTRigidBody* const rParent = dynamic_cast<const TNTRigidBody*>(parent);
	const TNTRigidBody* const rChild = dynamic_cast<const TNTRigidBody*>(child);
	RW_ASSERT(rParent || rChild);

	// Allocate new matrix and vector structures
	Eigen::MatrixXd::Index dimVel = 0;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		dimVel += constraint->getDimVelocity();
	}
	BOOST_FOREACH(const TNTContact* contact, contacts) {
		const VelocityScrew6D<>& velI = contact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> nij = contact->getNormalW(tntstate);
		const bool leaving = dot(-linRelVel,nij) >= 0;

		dimVel += 1;
		if (!leaving) {
			const TNTRestitutionModel::Values restitution = restitutionModel.getRestitution(*contact,tntstate,rwstate);
			if (restitution.enableTangent)
				dimVel += 2;
			if (restitution.enableAngular)
				dimVel += 1;
		}
	}
	Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(dimVel,dimVel);
	Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dimVel);

	dimVel = 0;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const VelocityScrew6D<>& velI = constraint->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(tntstate,rwstate);
		const Rotation3D<> linRot = constraint->getLinearRotationParentW(tntstate);
		const Rotation3D<> angRot = constraint->getAngularRotationParentW(tntstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();

		const Eigen::VectorXd::Index dim = constraint->getDimVelocity();
		if (dim > 0) {
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			// Construct RHS
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					rhs[dimVel+i] = -dot(linRelVel,linRot.getCol(i));
				}
			}
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i+3] == TNTConstraint::Velocity) {
					rhs[dimVel+3+i] = -dot(angRelVel,angRot.getCol(i));
				}
			}
			// Construct LHS blocks
			Eigen::MatrixXd::Index dimVelB = 0;
			BOOST_FOREACH(const TNTConstraint* constraintB, constraints) {
				const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
				if (dimB > 0) {
					lhs.block(dimVel,dimVelB,dim,dimB) = getBlock(constraint,constraintB,tntstate,rwstate);
					dimVelB += dimB;
				}
			}
			if (contacts.size() > 0)
				RW_THROW("TNTCollisionSolverSingle (solve): can not yet handle mixed contacts and constraints between two objects.");
			dimVel += dim;
		}
	}
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		const VelocityScrew6D<>& velI = contact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
		const Vector3D<> nij = contact->getNormalW(tntstate);
		const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
		const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
		const bool leaving = dot(-linRelVel,nij) >= 0;

		if (constraints.size() > 0)
			RW_THROW("TNTCollisionSolverSingle (solve): can not yet handle mixed contacts and constraints between two objects.");

		Eigen::VectorXd::Index dim = 1;
		// Construct RHS
		if (leaving) {
			rhs[dimVel] = -dot(linRelVel,nij);
		} else {
			const TNTRestitutionModel::Values restitution = restitutionModel.getRestitution(*contact,tntstate,rwstate);
			if (restitution.enableTangent)
				dim += 2;
			if (restitution.enableAngular)
				dim += 1;

			if (!restitution.enableTangent && !restitution.enableAngular) {
				rhs[dimVel] = -(1+restitution.normal)*dot(linRelVel,nij);
			} else if (restitution.enableTangent && !restitution.enableAngular) {
				rhs[dimVel+0] = -(1+restitution.tangent)*dot(linRelVel,tangentDir);
				rhs[dimVel+1] = -dot(linRelVel,zeroDir);
				rhs[dimVel+2] = -(1+restitution.normal)*dot(linRelVel,nij);
			} else if (!restitution.enableTangent && restitution.enableAngular) {
				rhs[dimVel+0] = -(1+restitution.normal)*dot(linRelVel,nij);
				rhs[dimVel+1] = -(1+restitution.angular)*dot(angRelVel,nij);
			} else if (restitution.enableTangent && restitution.enableAngular) {
				rhs[dimVel+0] = -(1+restitution.tangent)*dot(linRelVel,tangentDir);
				rhs[dimVel+1] = -dot(linRelVel,zeroDir);
				rhs[dimVel+2] = -(1+restitution.normal)*dot(linRelVel,nij);
				rhs[dimVel+3] = -(1+restitution.angular)*dot(angRelVel,nij);
			}
		}
		// Construct LHS blocks
		Eigen::MatrixXd::Index dimVelB = 0;
		BOOST_FOREACH(const TNTContact* contactB, contacts) {
			const Eigen::MatrixXd block = getBlock(contact,contactB,restitutionModel,tntstate,rwstate);
			RW_ASSERT(block.rows() == dim);
			const Eigen::MatrixXd::Index dimB = block.cols();
			lhs.block(dimVel,dimVelB,dim,dimB) = block;
			dimVelB += dimB;
		}
		dimVel += dim;
	}
	RW_ASSERT(lhs.rows() == lhs.cols());

	// Solve
	TNT_DEBUG_BOUNCING("Solving " << lhs.rows() << " x " << lhs.cols() << " equation system.");
	const Eigen::VectorXd sol = LinearAlgebra::pseudoInverse(lhs)*rhs;
	TNT_DEBUG_BOUNCING("Residual: " << (rhs-lhs*sol).transpose() << ".");

	// Apply the solution
	dimVel = 0;
	std::list<std::pair<Vector3D<>, Wrench6D<> > > parentImpulses;
	std::list<std::pair<Vector3D<>, Wrench6D<> > > childImpulses;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const Eigen::VectorXd::Index dim = constraint->getDimVelocity();
		const TNTBody* const cParent = constraint->getParent();
		const TNTBody* const cChild = constraint->getChild();
		const TNTRigidBody* const rcParent = dynamic_cast<const TNTRigidBody*>(cParent);
		const TNTRigidBody* const rcChild = dynamic_cast<const TNTRigidBody*>(cChild);

		if (dim > 0) {
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			const Rotation3D<> linRot = constraint->getLinearRotationParentW(tntstate);
			const Rotation3D<> angRot = constraint->getAngularRotationParentW(tntstate);

			Vector3D<> impF = Vector3D<>::zero();
			Vector3D<> impN = Vector3D<>::zero();
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					impF += sol[dimVel]*linRot.getCol(i);
					dimVel++;
				}
			}
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i+3] == TNTConstraint::Velocity) {
					impN += sol[dimVel]*angRot.getCol(i);
					dimVel++;
				}
			}

			const Vector3D<> rij = constraint->getPositionParentW(tntstate);
			const Vector3D<> rji = constraint->getPositionChildW(tntstate);
			if (rParent) {
				if (rParent == rcParent)
					parentImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(impF,impN)));
				else
					parentImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(-impF,-impN)));
			}
			if (rChild) {
				if (rChild == rcChild)
					childImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(-impF,-impN)));
				else
					childImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(impF,impN)));
			}
		}
	}
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		const TNTBody* const cParent = contact->getParent();
		const TNTBody* const cChild = contact->getChild();
		const TNTRigidBody* const rcParent = dynamic_cast<const TNTRigidBody*>(cParent);
		const TNTRigidBody* const rcChild = dynamic_cast<const TNTRigidBody*>(cChild);

		const VelocityScrew6D<>& velI = contact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> nij = contact->getNormalW(tntstate);
		const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
		const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
		const bool leaving = dot(-linRelVel,nij) >= 0;

		TNTRestitutionModel::Values restitution;
		if (!leaving)
			restitution = restitutionModel.getRestitution(*contact,tntstate,rwstate);

		Vector3D<> impF = Vector3D<>::zero();
		Vector3D<> impN = Vector3D<>::zero();
		if (!leaving && restitution.enableTangent) {
			impF += sol[dimVel]*tangentDir;
			dimVel++;
			impF += sol[dimVel]*zeroDir;
			dimVel++;
		}
		impF += sol[dimVel]*nij;
		dimVel++;
		if (!leaving && restitution.enableAngular) {
			impN = sol[dimVel]*nij;
			dimVel++;
		}

		const Vector3D<> rij = contact->getPositionParentW(tntstate);
		const Vector3D<> rji = contact->getPositionChildW(tntstate);
		if (rParent) {
			if (rParent == rcParent)
				parentImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(impF,impN)));
			else
				parentImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(-impF,-impN)));
		}
		if (rChild) {
			if (rChild == rcChild)
				childImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(-impF,-impN)));
			else
				childImpulses.push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(impF,impN)));
		}
	}
	typedef std::pair<Vector3D<>, Wrench6D<> > Impulse;
	if (rParent) {
		BOOST_FOREACH(const Impulse& impulse, parentImpulses) {
			TNTCollisionSolverSingle::applyImpulse(impulse.second, impulse.first, *rParent, tntstate);
		}
	}
	if (rChild) {
		BOOST_FOREACH(const Impulse& impulse, childImpulses) {
			TNTCollisionSolverSingle::applyImpulse(impulse.second, impulse.first, *rChild, tntstate);
		}
	}
}

Eigen::MatrixXd TNTCollisionSolverSingle::getBlock(
		const TNTConstraint* constraintA,
		const TNTConstraint* constraintB,
		const TNTIslandState& tntstate,
		const State& rwstate) const
{
	const TNTBody* const parentA = constraintA->getParent();
	const TNTBody* const childA = constraintA->getChild();
	const TNTBody* const parentB = constraintB->getParent();
	const TNTBody* const childB = constraintB->getChild();
	const TNTRigidBody* const rParentA = dynamic_cast<const TNTRigidBody*>(parentA);
	const TNTRigidBody* const rChildA = dynamic_cast<const TNTRigidBody*>(childA);
	const TNTRigidBody* const rParentB = dynamic_cast<const TNTRigidBody*>(parentB);
	const TNTRigidBody* const rChildB = dynamic_cast<const TNTRigidBody*>(childB);
	int sign;
	if (rParentA == rParentB && rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 6x6 matrix in world coordinates
	const Vector3D<> rij = constraintA->getPositionParentW(tntstate);
	const Vector3D<> rji = constraintA->getPositionChildW(tntstate);
	const Transform3D<>& wTbI = parentA->getWorldTcom(tntstate);
	const Transform3D<>& wTbJ = childA->getWorldTcom(tntstate);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();
	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = constraintB->getPositionParentW(tntstate);
	else
		rik = constraintB->getPositionChildW(tntstate);
	if (childA == parentB)
		rjk = constraintB->getPositionParentW(tntstate);
	else
		rjk = constraintB->getPositionChildW(tntstate);

	Eigen::Matrix3d BFlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d bNlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d BFang = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d bNang = Eigen::Matrix3d::Zero();
	if (rParentA) {
		const rw::common::Ptr<const RigidBody> rwbodyI = rParentA->getRigidBody();
		const InertiaMatrix<>& inertiaInvI = wTbI.R()*rwbodyI->getBodyInertiaInv()*inverse(wTbI.R());
		const double massInvI = rwbodyI->getMassInv();
		BFlin += sign*massInvI*Eigen::Matrix3d::Identity()-sign*Math::skew(rij-RI)*inertiaInvI.e()*Math::skew(rik-RI);
		bNlin += -sign*Math::skew(rij-RI)*inertiaInvI.e();
		BFang += sign*inertiaInvI.e()*Math::skew(rik-RI);
		bNang += sign*inertiaInvI.e();
	}
	if (rChildA) {
		const rw::common::Ptr<const RigidBody> rwbodyJ = rChildA->getRigidBody();
		const InertiaMatrix<>& inertiaInvJ = wTbJ.R()*rwbodyJ->getBodyInertiaInv()*inverse(wTbJ.R());
		const double massInvJ = rwbodyJ->getMassInv();
		BFlin += sign*massInvJ*Eigen::Matrix3d::Identity()-sign*Math::skew(rji-RJ)*inertiaInvJ.e()*Math::skew(rjk-RJ);
		bNlin += -sign*Math::skew(rji-RJ)*inertiaInvJ.e();
		BFang += sign*inertiaInvJ.e()*Math::skew(rjk-RJ);
		bNang += sign*inertiaInvJ.e();
	}

	// Now make rotated block of correct size
	const std::vector<TNTConstraint::Mode> modesA = constraintA->getConstraintModes();
	const std::vector<TNTConstraint::Mode> modesB = constraintB->getConstraintModes();
	const Rotation3D<> linRotA = constraintA->getLinearRotationParentW(tntstate);
	const Rotation3D<> angRotA = constraintA->getAngularRotationParentW(tntstate);
	const Rotation3D<> linRotB = constraintB->getLinearRotationParentW(tntstate);
	const Rotation3D<> angRotB = constraintB->getAngularRotationParentW(tntstate);

	const Eigen::VectorXd::Index dimA = constraintA->getDimVelocity();
	const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	for (std::size_t i = 0; i < 3; i++) {
		if (modesA[i] == TNTConstraint::Velocity) {
			col = 0;
			for (std::size_t j = 0; j < 3; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*linRotB.getCol(j).e();
					col++;
				}
			}
			for (std::size_t j = 3; j < 6; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*angRotB.getCol(j-3).e();
					col++;
				}
			}
			row++;
		}
	}
	for (std::size_t i = 3; i < 6; i++) {
		if (modesA[i] == TNTConstraint::Velocity) {
			col = 0;
			for (std::size_t j = 0; j < 3; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = angRotA.getCol(i-3).e().transpose()*BFlin*linRotB.getCol(j).e();
					col++;
				}
			}
			for (std::size_t j = 3; j < 6; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = angRotA.getCol(i-3).e().transpose()*BFlin*angRotB.getCol(j-3).e();
					col++;
				}
			}
			row++;
		}
	}

	return block;
}

Eigen::MatrixXd TNTCollisionSolverSingle::getBlock(
		const TNTContact* contactA,
		const TNTContact* contactB,
		const TNTRestitutionModel& restitutionModel,
		const TNTIslandState& tntstate,
		const State& rwstate) const
{
	const TNTBody* const parentA = contactA->getParent();
	const TNTBody* const childA = contactA->getChild();
	const TNTBody* const parentB = contactB->getParent();
	const TNTBody* const childB = contactB->getChild();
	const TNTRigidBody* const rParentA = dynamic_cast<const TNTRigidBody*>(parentA);
	const TNTRigidBody* const rChildA = dynamic_cast<const TNTRigidBody*>(childA);
	const TNTRigidBody* const rParentB = dynamic_cast<const TNTRigidBody*>(parentB);
	const TNTRigidBody* const rChildB = dynamic_cast<const TNTRigidBody*>(childB);
	int sign;
	if (rParentA == rParentB && rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 4x4 matrix in world coordinates
	const VelocityScrew6D<>& velI = contactA->getVelocityParentW(tntstate,rwstate);
	const VelocityScrew6D<>& velJ = contactA->getVelocityChildW(tntstate,rwstate);
	const Vector3D<> linRelVel = velI.linear()-velJ.linear();
	const VelocityScrew6D<>& velIB = contactB->getVelocityParentW(tntstate,rwstate);
	const VelocityScrew6D<>& velJB = contactB->getVelocityChildW(tntstate,rwstate);
	const Vector3D<> linRelVelB = velIB.linear()-velJB.linear();
	const Vector3D<> rij = contactA->getPositionParentW(tntstate);
	const Vector3D<> rji = contactA->getPositionChildW(tntstate);
	const Vector3D<> nij = contactA->getNormalW(tntstate);
	const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
	const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
	const Vector3D<> nijB = contactB->getNormalW(tntstate);
	const Vector3D<> zeroDirB = normalize(cross(nijB,linRelVelB));
	const Vector3D<> tangentDirB = normalize(cross(zeroDirB,nijB));
	const Transform3D<>& wTbI = parentA->getWorldTcom(tntstate);
	const Transform3D<>& wTbJ = childA->getWorldTcom(tntstate);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();
	const bool leavingA = dot(-linRelVel,nij) >= 0;
	const bool leavingB = dot(-linRelVelB,nijB) >= 0;

	TNTRestitutionModel::Values restitutionA;
	TNTRestitutionModel::Values restitutionB;
	if (!leavingA)
		restitutionA = restitutionModel.getRestitution(*contactA,tntstate,rwstate);
	if (!leavingB)
		restitutionB = restitutionModel.getRestitution(*contactB,tntstate,rwstate);

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = contactB->getPositionParentW(tntstate);
	else
		rik = contactB->getPositionChildW(tntstate);
	if (childA == parentB)
		rjk = contactB->getPositionParentW(tntstate);
	else
		rjk = contactB->getPositionChildW(tntstate);

	Eigen::Matrix3d BFlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix<double,3,1> bNlin = Eigen::Matrix<double,3,1>::Zero();
	Eigen::Matrix<double,1,3> BFang = Eigen::Matrix<double,1,3>::Zero();
	double bNang = 0;
	if (rParentA) {
		const rw::common::Ptr<const RigidBody> rwbodyI = rParentA->getRigidBody();
		const InertiaMatrix<>& inertiaInvI = wTbI.R()*rwbodyI->getBodyInertiaInv()*inverse(wTbI.R());
		const double massInvI = rwbodyI->getMassInv();
		BFlin += sign*massInvI*Eigen::Matrix3d::Identity()-sign*Math::skew(rij-RI)*inertiaInvI.e()*Math::skew(rik-RI);
		if (!leavingB && restitutionB.enableAngular)
			bNlin += -sign*Math::skew(rij-RI)*inertiaInvI.e()*nij.e();
		if (!leavingA && restitutionA.enableAngular)
			BFang += sign*nij.e().transpose()*inertiaInvI.e()*Math::skew(rik-RI);
		if (!leavingA && !leavingB && restitutionA.enableAngular && restitutionB.enableAngular)
			bNang += sign*nij.e().transpose()*inertiaInvI.e()*nij.e();
	}
	if (rChildA) {
		const rw::common::Ptr<const RigidBody> rwbodyJ = rChildA->getRigidBody();
		const InertiaMatrix<>& inertiaInvJ = wTbJ.R()*rwbodyJ->getBodyInertiaInv()*inverse(wTbJ.R());
		const double massInvJ = rwbodyJ->getMassInv();
		BFlin += sign*massInvJ*Eigen::Matrix3d::Identity()-sign*Math::skew(rji-RJ)*inertiaInvJ.e()*Math::skew(rjk-RJ);
		if (!leavingB && restitutionB.enableAngular)
			bNlin += -sign*Math::skew(rji-RJ)*inertiaInvJ.e()*nij.e();
		if (!leavingA && restitutionA.enableAngular)
			BFang += sign*nij.e().transpose()*inertiaInvJ.e()*Math::skew(rjk-RJ);
		if (!leavingA && !leavingB && restitutionA.enableAngular && restitutionB.enableAngular)
			bNang += sign*nij.e().transpose()*inertiaInvJ.e()*nij.e();
	}

	// Now make rotated block of correct size
	Eigen::VectorXd::Index dimA = 1;
	Eigen::VectorXd::Index dimB = 1;
	if (!leavingA && restitutionA.enableTangent)
		dimA += 2;
	if (!leavingB && restitutionB.enableTangent)
		dimB += 2;
	if (!leavingA && restitutionA.enableAngular)
		dimA += 1;
	if (!leavingB && restitutionB.enableAngular)
		dimB += 1;
	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	if (!leavingA && restitutionA.enableTangent) {
		if (!leavingB && restitutionB.enableTangent) {
			block(row+0,col) = tangentDir.e().transpose()*BFlin*tangentDirB.e();
			block(row+1,col) = zeroDir.e().transpose()*BFlin*tangentDirB.e();
			col++;
			block(row+0,col) = tangentDir.e().transpose()*BFlin*zeroDirB.e();
			block(row+1,col) = zeroDir.e().transpose()*BFlin*zeroDirB.e();
			col++;
		}
		block(row+0,col) = tangentDir.e().transpose()*BFlin*nijB.e();
		block(row+1,col) = zeroDir.e().transpose()*BFlin*nijB.e();
		col++;
		if (!leavingB && restitutionB.enableAngular) {
			block(row+0,col) = tangentDir.e().transpose()*bNlin;
			block(row+1,col) = zeroDir.e().transpose()*bNlin;
			col++;
		}
		row += 2;
	}
	col = 0;
	{
		if (!leavingB && restitutionB.enableTangent) {
			block(row,col) = nij.e().transpose()*BFlin*tangentDirB.e();
			col++;
			block(row,col) = nij.e().transpose()*BFlin*zeroDirB.e();
			col++;
		}
		block(row,col) = nij.e().transpose()*BFlin*nijB.e();
		col++;
		if (!leavingB && restitutionB.enableAngular) {
			block(row,col) = nij.e().transpose()*bNlin;
			col++;
		}
		row++;
	}
	col = 0;
	if (!leavingA && restitutionA.enableAngular) {
		if (!leavingB && restitutionB.enableTangent) {
			block(row,col) = BFang*tangentDirB.e();
			col++;
			block(row,col) = BFang*zeroDirB.e();
			col++;
		}
		block(row,col) = BFang*nijB.e();
		col++;
		if (!leavingB && restitutionB.enableAngular) {
			block(row,col) = bNang;
			col++;
		}
		row++;
	}
	return block;
}
