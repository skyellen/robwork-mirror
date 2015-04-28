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

#include "TNTConstraintCorrection.hpp"
#include "TNTSettings.hpp"
#include "TNTConstraint.hpp"
#include "TNTIslandState.hpp"

#include "TNTRigidBody.hpp"
#include "TNTContact.hpp"

#include <rwsim/dynamics/Body.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_PROPTHRESHOLDFACTOR "TNTCorrectionThresholdFactor"
#define PROPERTY_PROPLAYER "TNTCorrectionContactLayer"

struct TNTConstraintCorrection::BodyPairError {
	BodyPairError(): body1(NULL), body2(NULL), error(0) {}
	BodyPairError(const TNTBody* body1, const TNTBody* body2, double error):
		body1(body1),
		body2(body2),
		error(error)
	{}
	const TNTBody* body1;
	const TNTBody* body2;
	double error;
};

TNTConstraintCorrection::TNTConstraintCorrection() {
}

TNTConstraintCorrection::~TNTConstraintCorrection() {
}

void TNTConstraintCorrection::correct(const std::list<TNTConstraint*>& constraints, TNTIslandState& tntstate, const PropertyMap& properties) const {
	double threshold = properties.get<double>(PROPERTY_PROPTHRESHOLDFACTOR, -1);
	double layer = properties.get<double>(PROPERTY_PROPLAYER, -1);
	if (threshold < 0 || layer < 0) {
		PropertyMap map;
		addDefaultProperties(map);
		if (threshold < 0)
			threshold = map.get<double>(PROPERTY_PROPTHRESHOLDFACTOR, -1);
		if (layer < 0)
			layer = map.get<double>(PROPERTY_PROPLAYER, -1);
	}
	RW_ASSERT(threshold > 0);
	RW_ASSERT(layer > 0);

	std::list<const TNTConstraint*> reducedConstraints;
	unsigned int nrOfConstraints = 0;
	std::map<const TNTBody*, unsigned int> bodies;
	std::vector<const TNTRigidBody*> idToBody;
	std::list<BodyPairError> idsToError;
	unsigned int id = 0;
	for (std::list<TNTConstraint*>::const_iterator itA = constraints.begin(); itA != constraints.end(); itA++) {
		const TNTConstraint* const constraint = *itA;
		if (constraint->getDimVelocity() == 0)
			continue;
		const TNTRigidBody* const rParent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const TNTRigidBody* const rChild = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		if (!rParent && !rChild)
			continue;
		const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
		if (rParent) {
			if (bodies.find(rParent) == bodies.end()) {
				bodies[rParent] = id;
				idToBody.push_back(rParent);
				id++;
			}
		}
		if (rChild) {
			if (bodies.find(rChild) == bodies.end()) {
				bodies[rChild] = id;
				idToBody.push_back(rChild);
				id++;
			}
		}
		if (contact) {
			bool match = false;
			/*std::list<TNTConstraint*>::const_iterator itB = itA;
			for (itB++; itB != constraints.end(); itB++) {
				const TNTConstraint* const constraintB = *itB;
				const TNTContact* const contactB = dynamic_cast<const TNTContact*>(constraintB);
				if (!contactB)
					continue;
				if ((constraint->getParent() != contactB->getParent() || constraint->getChild() != contactB->getChild()) &&
						(constraint->getParent() != contactB->getChild() || constraint->getChild() != contactB->getParent()))
					continue;
				const Vector3D<> nA = contact->getNormalW(tntstate);
				const Vector3D<> pAp = contact->getPositionParentW(tntstate);
				const Vector3D<> pAc = contact->getPositionChildW(tntstate);
				Vector3D<> nB;
				Vector3D<> pBp;
				Vector3D<> pBc;
				if (constraint->getParent() == contactB->getParent()) {
					nB = contactB->getNormalW(tntstate);
					pBp = contactB->getPositionParentW(tntstate);
					pBc = contactB->getPositionChildW(tntstate);
				} else {
					nB = -contactB->getNormalW(tntstate);
					pBp = contactB->getPositionChildW(tntstate);
					pBc = contactB->getPositionParentW(tntstate);
				}
				const double distP = (pAp-pBp).norm2();
				const double distC = (pAc-pBc).norm2();
				const double distN = acos(dot(nA,nB));
				if (distN > std::max(distP,distC)*threshold*Deg2Rad) {
					match = true;
					break;
				}
			}*/
			if (!match) {
				nrOfConstraints += 1;
				reducedConstraints.push_back(constraint);
			}

			// Save initial error to check if it is getting smaller
			const double error = -contact->getContact().getDepth();
			idsToError.push_back(BodyPairError(contact->getParent(),contact->getChild(),error));
		} else {
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			for (std::size_t i = 0; i < 6; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					nrOfConstraints++;
				}
			}
			reducedConstraints.push_back(constraint);
		}
	}

	if (nrOfConstraints == 0 || id == 0)
		return;

	unsigned int curConstraint = 0;
	Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(nrOfConstraints,6*id);
	Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nrOfConstraints);
	BOOST_FOREACH(const TNTConstraint* const constraint, reducedConstraints) {
		const TNTRigidBody* const rParent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const TNTRigidBody* const rChild = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
		if (contact) {
			const Vector3D<> nij = contact->getNormalW(tntstate);
			rhs[curConstraint] = -contact->getContact().getDepth();
			if (rParent) {
				const Vector3D<> Ri = rParent->getWorldTcom(tntstate).P();
				const Vector3D<> rij = contact->getPositionParentW(tntstate);
				const unsigned int bodyId = bodies[rParent]*6;
				lhs(curConstraint,bodyId+0) = nij[0];
				lhs(curConstraint,bodyId+1) = nij[1];
				lhs(curConstraint,bodyId+2) = nij[2];
				const Vector3D<> ang(-Math::skew(rij-Ri).transpose()*nij.e());
				lhs(curConstraint,bodyId+3) = ang[0];
				lhs(curConstraint,bodyId+4) = ang[1];
				lhs(curConstraint,bodyId+5) = ang[2];
			}
			if (rChild) {
				const Vector3D<> Rj = rChild->getWorldTcom(tntstate).P();
				const Vector3D<> rji = contact->getPositionChildW(tntstate);
				const unsigned int bodyId = bodies[rChild]*6;
				lhs(curConstraint,bodyId+0) = -nij[0];
				lhs(curConstraint,bodyId+1) = -nij[1];
				lhs(curConstraint,bodyId+2) = -nij[2];
				const Vector3D<> ang(Math::skew(rji-Rj).transpose()*nij.e());
				lhs(curConstraint,bodyId+3) = ang[0];
				lhs(curConstraint,bodyId+4) = ang[1];
				lhs(curConstraint,bodyId+5) = ang[2];
			}
			curConstraint += 1;
		} else {
			const Vector3D<> rij = constraint->getPositionParentW(tntstate);
			const Vector3D<> rji = constraint->getPositionChildW(tntstate);
			const Rotation3D<> rotI = constraint->getAngularRotationParentW(tntstate);
			const Rotation3D<> rotJ = constraint->getAngularRotationChildW(tntstate);
			const EAA<> rotDifEAA(rotJ*inverse(rotI));
			const Vector3D<> rotDif = rotDifEAA.angle()*rotDifEAA.axis();
			const Rotation3D<> linR = constraint->getLinearRotationParentW(tntstate);
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			std::size_t constraintID = curConstraint;
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					const Vector3D<> dir = linR.getCol(i);
					rhs[constraintID] = dot(rji-rij,dir);
					constraintID++;
				}
			}
			for (std::size_t i = 3; i < 6; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					const Vector3D<> dir = rotI.getCol(i-3);
					rhs[constraintID] = dot(rotDif,dir);
					constraintID++;
				}
			}
			if (rParent) {
				const Vector3D<> Ri = rParent->getWorldTcom(tntstate).P();
				const unsigned int bodyId = bodies[rParent]*6;
				constraintID = curConstraint;
				for (std::size_t i = 0; i < 3; i++) {
					if (modes[i] == TNTConstraint::Velocity) {
						const Vector3D<> dir = linR.getCol(i);
						lhs(constraintID,bodyId+0) = dir[0];
						lhs(constraintID,bodyId+1) = dir[1];
						lhs(constraintID,bodyId+2) = dir[2];
						const Vector3D<> ang(-Math::skew(rij-Ri).transpose()*dir.e());
						lhs(constraintID,bodyId+3) = ang[0];
						lhs(constraintID,bodyId+4) = ang[1];
						lhs(constraintID,bodyId+5) = ang[2];
						constraintID++;
					}
				}
				for (std::size_t i = 3; i < 6; i++) {
					if (modes[i] == TNTConstraint::Velocity) {
						lhs(constraintID,bodyId+0) = 0;
						lhs(constraintID,bodyId+1) = 0;
						lhs(constraintID,bodyId+2) = 0;
						const Vector3D<> dir = rotI.getCol(i-3);
						lhs(constraintID,bodyId+3) = dir[0];
						lhs(constraintID,bodyId+4) = dir[1];
						lhs(constraintID,bodyId+5) = dir[2];
						constraintID++;
					}
				}
			}
			if (rChild) {
				const Vector3D<> Rj = rChild->getWorldTcom(tntstate).P();
				const unsigned int bodyId = bodies[rChild]*6;
				constraintID = curConstraint;
				for (std::size_t i = 0; i < 3; i++) {
					if (modes[i] == TNTConstraint::Velocity) {
						const Vector3D<> dir = linR.getCol(i);
						lhs(constraintID,bodyId+0) = -dir[0];
						lhs(constraintID,bodyId+1) = -dir[1];
						lhs(constraintID,bodyId+2) = -dir[2];
						const Vector3D<> ang(Math::skew(rji-Rj).transpose()*dir.e());
						lhs(constraintID,bodyId+3) = ang[0];
						lhs(constraintID,bodyId+4) = ang[1];
						lhs(constraintID,bodyId+5) = ang[2];
						constraintID++;
					}
				}
				for (std::size_t i = 3; i < 6; i++) {
					if (modes[i] == TNTConstraint::Velocity) {
						lhs(constraintID,bodyId+0) = 0;
						lhs(constraintID,bodyId+1) = 0;
						lhs(constraintID,bodyId+2) = 0;
						const Vector3D<> dir = -rotI.getCol(i-3);
						lhs(constraintID,bodyId+3) = dir[0];
						lhs(constraintID,bodyId+4) = dir[1];
						lhs(constraintID,bodyId+5) = dir[2];
						constraintID++;
					}
				}
			}
			for (std::size_t i = 0; i < 6; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					curConstraint++;
				}
			}
		}
	}

	// Skip correction if it is small anyway
	/*bool doSolve = false;
	for (unsigned int i = 0; i < nrOfConstraints; i++) {
		if (fabs((double)rhs[i]) > 1e-8) {
			doSolve = true;
			break;
		}
	}
	if (!doSolve)
		return;*/

	//const Eigen::MatrixXd lhsInv = LinearAlgebra::pseudoInverse(lhs,1e-6);
	//RW_ASSERT(lhsInv.cols() == rhs.rows());
	//const Eigen::VectorXd sol = lhsInv*rhs;
	const Eigen::VectorXd sol = minimize(lhs,rhs,1e-9,layer);
	for (unsigned int i = 0; i < id; i++) {
		const Eigen::VectorXd correction = sol.block(i*6,0,6,1);
		const TNTRigidBody* body = idToBody[i];
		Transform3D<> wTcom = body->getWorldTcom(tntstate);
		wTcom.P() += Vector3D<>(correction.block(0,0,3,1));
		wTcom.R() = EAA<>(Vector3D<>(correction.block(3,0,3,1))).toRotation3D()*wTcom.R();
		body->setWorldTcom(wTcom,tntstate);
	}

	// Construct map from body pair to error
	std::vector<std::vector<double> > contactErrors(idToBody.size(),std::vector<double>(idToBody.size(),0));
	BOOST_FOREACH(const BodyPairError& bpe, idsToError) {
		const unsigned int id1 = bodies[bpe.body1];
		const unsigned int id2 = bodies[bpe.body2];
		const double err = fabs(bpe.error);
		if (err > contactErrors[id1][id2]) {
			contactErrors[id1][id2] = err;
			contactErrors[id2][id1] = err;
		}
	}

	BOOST_FOREACH(const TNTConstraint* const constraint, reducedConstraints) {
		const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
		if (contact) {
			const double newError = -contact->getContact().getDepth();
			const unsigned int id1 = bodies[contact->getParent()];
			const unsigned int id2 = bodies[contact->getChild()];
			const double error = contactErrors[id1][id2];
			if (fabs(newError) > error)
				RW_THROW("TNTConstraintCorrection (correct): correction failed - error between " << contact->getParent()->get()->getName() << " and " << contact->getChild()->get()->getName() << " increased from " << error << " to " << newError << ".");
		}
	}
}

Eigen::VectorXd TNTConstraintCorrection::minimize(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double svdPrecision, double layer) {
	const Eigen::MatrixXd::Index M = A.rows();
	const Eigen::MatrixXd::Index N = A.cols();

	Eigen::MatrixXd B(M,M+N);
	B.block(0,0,M,N) = A;
	Eigen::VectorXd x(N);

	double alpha = 1;
	bool iterate = true;
	unsigned int k = 0;
	for (; k < 10 && iterate; k++) {
		iterate = false;
		B.block(0,N,M,M) = Eigen::MatrixXd::Identity(M,M)*(-1./sqrt(alpha));
		const Eigen::MatrixXd Binv = LinearAlgebra::pseudoInverse(B,svdPrecision);
		const Eigen::VectorXd y = Binv*b;
		x = y.topRows(N);
		const Eigen::VectorXd s = y.bottomRows(M)/sqrt(alpha);

		TNT_DEBUG_CORRECTION("Alpha: " << alpha);
		TNT_DEBUG_CORRECTION(" - Contact/constraint error: " << s.transpose());
		TNT_DEBUG_CORRECTION(" - Displacement: " << x.transpose());

		if (s.maxCoeff() < -layer) {
			alpha *= 2;
			iterate = true;
		}
	}

	TNT_DEBUG_CORRECTION("Iterations: " << k << " - alpha: " << alpha);

	return x;
}

void TNTConstraintCorrection::addDefaultProperties(PropertyMap& properties) {
	properties.add<double>(PROPERTY_PROPTHRESHOLDFACTOR,"Angle between a pair of contact normals must be less than this factor multiplied by the distance between the contact points to be included (degrees per meter).",10.0/0.003);
	properties.add<double>(PROPERTY_PROPLAYER,"The maximum penetration allowed in a contact after correction (in meters).",0.0005);
}
