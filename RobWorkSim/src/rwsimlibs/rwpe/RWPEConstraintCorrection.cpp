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

#include <rwsim/dynamics/Body.hpp>
#include "RWPEConstraintCorrection.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEConstraint.hpp"
#include "RWPEContact.hpp"
#include "RWPEIslandState.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

//#define PROPERTY_PROPTHRESHOLDFACTOR "RWPECorrectionThresholdFactor"
#define PROPERTY_PROPLAYER "RWPECorrectionContactLayer"

RWPEConstraintCorrection::RWPEConstraintCorrection() {
}

RWPEConstraintCorrection::~RWPEConstraintCorrection() {
}

void RWPEConstraintCorrection::correct(const std::list<RWPEConstraint*>& constraints, RWPEIslandState& state, const PropertyMap& properties, RWPELogUtil* log) const {
	//double threshold = properties.get<double>(PROPERTY_PROPTHRESHOLDFACTOR, -1);
	double layer = properties.get<double>(PROPERTY_PROPLAYER, -1);
	//if (threshold < 0 || layer < 0) {
	if (layer < 0) {
		PropertyMap map;
		addDefaultProperties(map);
		//if (threshold < 0)
		//	threshold = map.get<double>(PROPERTY_PROPTHRESHOLDFACTOR, -1);
		if (layer < 0)
			layer = map.get<double>(PROPERTY_PROPLAYER, -1);
	}
	//RW_ASSERT(threshold > 0);
	RW_ASSERT(layer > 0);

	const bool doLog = (log == NULL)? false : log->doLog();

	if (doLog) {
		std::vector<std::string> labels;
		std::vector<double> values;
		labels.push_back("Target Layer [m]");
		labels.push_back("Contacts & Constraints");
		values.push_back(layer);
		values.push_back(constraints.size());
		log->addValues("Correction Input",values,labels,RWPE_LOCATION);
	}

	std::list<const RWPEConstraint*> reducedConstraints;
	unsigned int nrOfConstraints = 0;
	unsigned int nrOfContacts = 0;
	std::map<const RWPEBody*, unsigned int> bodies;
	std::vector<const RWPEBodyDynamic*> idToBody;
	unsigned int id = 0;
	for (std::list<RWPEConstraint*>::const_iterator itA = constraints.begin(); itA != constraints.end(); itA++) {
		const RWPEConstraint* const constraint = *itA;
		if (constraint->getDimVelocity() == 0)
			continue;
		const RWPEBodyDynamic* const rParent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const RWPEBodyDynamic* const rChild = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		if (!rParent && !rChild)
			continue;
		const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
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
			nrOfConstraints += 1;
			nrOfContacts += 1;
			reducedConstraints.push_back(constraint);
		} else {
			const std::vector<RWPEConstraint::Mode> modes = constraint->getConstraintModes();
			for (std::size_t i = 0; i < 6; i++) {
				if (modes[i] == RWPEConstraint::Velocity) {
					nrOfConstraints++;
				}
			}
			reducedConstraints.push_back(constraint);
		}
	}

	if (nrOfConstraints == 0 || id == 0)
		return;

	unsigned int curConstraint = 0;
	unsigned int curContact = 0;
	Eigen::MatrixXd lhsE = Eigen::MatrixXd::Zero(nrOfConstraints,6*id);
	Eigen::VectorXd rhsE = Eigen::VectorXd::Zero(nrOfConstraints);
	Eigen::MatrixXd lhsI = Eigen::MatrixXd::Zero(nrOfContacts,6*id);
	Eigen::VectorXd rhsI = Eigen::VectorXd::Zero(nrOfContacts);
	std::vector<bool> hasContacts(id,false);
	BOOST_FOREACH(const RWPEConstraint* const constraint, reducedConstraints) {
		const RWPEBodyDynamic* const rParent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const RWPEBodyDynamic* const rChild = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
		if (contact) {
			const Vector3D<> nij = contact->getNormalW(state);
			rhsE[curConstraint] = -contact->getContact().getDepth();
			rhsI[curContact] = rhsE[curConstraint];
			if (rParent) {
				const Vector3D<> Ri = rParent->getWorldTcom(state).P();
				const Vector3D<> rij = contact->getPositionParentW(state);
				const unsigned int bodyId = bodies[rParent]*6;
				lhsE(curConstraint,bodyId+0) = nij[0];
				lhsE(curConstraint,bodyId+1) = nij[1];
				lhsE(curConstraint,bodyId+2) = nij[2];
				lhsI(curContact,bodyId+0) = nij[0];
				lhsI(curContact,bodyId+1) = nij[1];
				lhsI(curContact,bodyId+2) = nij[2];
				const Vector3D<> ang(-Math::skew(rij-Ri).transpose()*nij.e());
				lhsE(curConstraint,bodyId+3) = ang[0];
				lhsE(curConstraint,bodyId+4) = ang[1];
				lhsE(curConstraint,bodyId+5) = ang[2];
				lhsI(curContact,bodyId+3) = ang[0];
				lhsI(curContact,bodyId+4) = ang[1];
				lhsI(curContact,bodyId+5) = ang[2];
				hasContacts[bodyId] = true;
			}
			if (rChild) {
				const Vector3D<> Rj = rChild->getWorldTcom(state).P();
				const Vector3D<> rji = contact->getPositionChildW(state);
				const unsigned int bodyId = bodies[rChild]*6;
				lhsE(curConstraint,bodyId+0) = -nij[0];
				lhsE(curConstraint,bodyId+1) = -nij[1];
				lhsE(curConstraint,bodyId+2) = -nij[2];
				lhsI(curContact,bodyId+0) = -nij[0];
				lhsI(curContact,bodyId+1) = -nij[1];
				lhsI(curContact,bodyId+2) = -nij[2];
				const Vector3D<> ang(Math::skew(rji-Rj).transpose()*nij.e());
				lhsE(curConstraint,bodyId+3) = ang[0];
				lhsE(curConstraint,bodyId+4) = ang[1];
				lhsE(curConstraint,bodyId+5) = ang[2];
				lhsI(curContact,bodyId+3) = ang[0];
				lhsI(curContact,bodyId+4) = ang[1];
				lhsI(curContact,bodyId+5) = ang[2];
				hasContacts[bodyId] = true;
			}
			curConstraint += 1;
			curContact += 1;
		} else {
			const Vector3D<> rij = constraint->getPositionParentW(state);
			const Vector3D<> rji = constraint->getPositionChildW(state);
			const Rotation3D<> rotI = constraint->getAngularRotationParentW(state);
			const Rotation3D<> rotJ = constraint->getAngularRotationChildW(state);
			const EAA<> rotDifEAA(rotJ*inverse(rotI));
			const Vector3D<> rotDif = rotDifEAA.angle()*rotDifEAA.axis();
			const Rotation3D<> linR = constraint->getLinearRotationParentW(state);
			const std::vector<RWPEConstraint::Mode> modes = constraint->getConstraintModes();
			std::size_t constraintID = curConstraint;
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == RWPEConstraint::Velocity) {
					const Vector3D<> dir = linR.getCol(i);
					rhsE[constraintID] = dot(rji-rij,dir);
					constraintID++;
				}
			}
			for (std::size_t i = 3; i < 6; i++) {
				if (modes[i] == RWPEConstraint::Velocity) {
					const Vector3D<> dir = rotI.getCol(i-3);
					rhsE[constraintID] = dot(rotDif,dir);
					constraintID++;
				}
			}
			if (rParent) {
				const Vector3D<> Ri = rParent->getWorldTcom(state).P();
				const unsigned int bodyId = bodies[rParent]*6;
				constraintID = curConstraint;
				for (std::size_t i = 0; i < 3; i++) {
					if (modes[i] == RWPEConstraint::Velocity) {
						const Vector3D<> dir = linR.getCol(i);
						lhsE(constraintID,bodyId+0) = dir[0];
						lhsE(constraintID,bodyId+1) = dir[1];
						lhsE(constraintID,bodyId+2) = dir[2];
						const Vector3D<> ang(-Math::skew(rij-Ri).transpose()*dir.e());
						lhsE(constraintID,bodyId+3) = ang[0];
						lhsE(constraintID,bodyId+4) = ang[1];
						lhsE(constraintID,bodyId+5) = ang[2];
						constraintID++;
					}
				}
				for (std::size_t i = 3; i < 6; i++) {
					if (modes[i] == RWPEConstraint::Velocity) {
						lhsE(constraintID,bodyId+0) = 0;
						lhsE(constraintID,bodyId+1) = 0;
						lhsE(constraintID,bodyId+2) = 0;
						const Vector3D<> dir = rotI.getCol(i-3);
						lhsE(constraintID,bodyId+3) = dir[0];
						lhsE(constraintID,bodyId+4) = dir[1];
						lhsE(constraintID,bodyId+5) = dir[2];
						constraintID++;
					}
				}
			}
			if (rChild) {
				const Vector3D<> Rj = rChild->getWorldTcom(state).P();
				const unsigned int bodyId = bodies[rChild]*6;
				constraintID = curConstraint;
				for (std::size_t i = 0; i < 3; i++) {
					if (modes[i] == RWPEConstraint::Velocity) {
						const Vector3D<> dir = linR.getCol(i);
						lhsE(constraintID,bodyId+0) = -dir[0];
						lhsE(constraintID,bodyId+1) = -dir[1];
						lhsE(constraintID,bodyId+2) = -dir[2];
						const Vector3D<> ang(Math::skew(rji-Rj).transpose()*dir.e());
						lhsE(constraintID,bodyId+3) = ang[0];
						lhsE(constraintID,bodyId+4) = ang[1];
						lhsE(constraintID,bodyId+5) = ang[2];
						constraintID++;
					}
				}
				for (std::size_t i = 3; i < 6; i++) {
					if (modes[i] == RWPEConstraint::Velocity) {
						lhsE(constraintID,bodyId+0) = 0;
						lhsE(constraintID,bodyId+1) = 0;
						lhsE(constraintID,bodyId+2) = 0;
						const Vector3D<> dir = -rotI.getCol(i-3);
						lhsE(constraintID,bodyId+3) = dir[0];
						lhsE(constraintID,bodyId+4) = dir[1];
						lhsE(constraintID,bodyId+5) = dir[2];
						constraintID++;
					}
				}
			}
			for (std::size_t i = 0; i < 6; i++) {
				if (modes[i] == RWPEConstraint::Velocity) {
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

	//RWPE_DEBUG_CORRECTION("Error larger than 1e-8 - trying to solve.");
	if (doLog) {
		std::map<std::string,rw::math::Transform3D<> > positions;
		for (unsigned int i = 0; i < id; i++) {
			const RWPEBodyDynamic* const body = idToBody[i];
			const BodyInfo& info = body->get()->getInfo();
			Transform3D<> wTb = body->getWorldTcom(state);
			wTb.P() -= wTb.R()*info.masscenter;
			positions[body->get()->getName()] = wTb;
		}
		log->addPositions("Positions Before",positions,RWPE_LOCATION);

		std::vector<const RWPEContact*> contactsUsed;
		BOOST_FOREACH(const RWPEConstraint* const constraint, reducedConstraints) {
			const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
			if (contact)
				contactsUsed.push_back(contact);
		}
		log->addContacts("Contacts Used",contactsUsed,RWPE_LOCATION);

		std::ostream& lstr = log->log("Equation System",RWPE_LOCATION);
		lstr << "LHS equality: " << std::endl << lhsE << std::endl;
		lstr << "RHS equality:" << std::endl << rhsE.transpose();
		lstr << "LHS inequality: " << std::endl << lhsI << std::endl;
		lstr << "RHS inequality:" << std::endl << rhsI.transpose();
		std::vector<std::string> labels;
		std::vector<double> values;
		labels.push_back("Correction Equality Rows");
		labels.push_back("Correction Equality Columns");
		labels.push_back("Correction Inequality Rows");
		labels.push_back("Correction Inequality Columns");
		values.push_back(lhsE.rows());
		values.push_back(lhsE.cols());
		values.push_back(lhsI.rows());
		values.push_back(lhsI.cols());
		log->addValues("Correction",values,labels,RWPE_LOCATION);
	}

	//const Eigen::MatrixXd lhsInv = LinearAlgebra::pseudoInverse(lhs,1e-6);
	//RW_ASSERT(lhsInv.cols() == rhs.rows());
	//const Eigen::VectorXd sol = lhsInv*rhs;
	const Eigen::VectorXd sol = minimize(lhsE,rhsE,lhsI,rhsI,hasContacts,1e-9,layer,log);

	for (unsigned int i = 0; i < id; i++) {
		const Eigen::VectorXd correction = sol.block(i*6,0,6,1);
		const RWPEBodyDynamic* body = idToBody[i];
		Transform3D<> wTcom = body->getWorldTcom(state);
		wTcom.P() += Vector3D<>(correction.block(0,0,3,1));
		wTcom.R() = EAA<>(Vector3D<>(correction.block(3,0,3,1))).toRotation3D()*wTcom.R();
		body->setWorldTcom(wTcom,state);
	}

	if (doLog) {
		std::ostream& lstr = log->log("Solution",RWPE_LOCATION);
		lstr << "Solution: " << std::endl << sol.transpose();
		lstr << std::endl;
		lstr << "Residual (equalities):" << std::endl << (lhsE*sol-rhsE).transpose();

		std::map<std::string,rw::math::Transform3D<> > positions;
		for (unsigned int i = 0; i < id; i++) {
			const RWPEBodyDynamic* const body = idToBody[i];
			const BodyInfo& info = body->get()->getInfo();
			Transform3D<> wTb = body->getWorldTcom(state);
			wTb.P() -= wTb.R()*info.masscenter;
			positions[body->get()->getName()] = wTb;
		}
		log->addPositions("Positions After",positions,RWPE_LOCATION);
	}
}

Eigen::VectorXd RWPEConstraintCorrection::minimize(
		const Eigen::MatrixXd& Ae,
		const Eigen::VectorXd& be,
		const Eigen::MatrixXd& Ai,
		const Eigen::VectorXd& bi,
		const std::vector<bool>& hasContacts,
		double svdPrecision, double layer, RWPELogUtil* log)
{
	RW_ASSERT(Ae.cols() == Ai.cols());

	const bool doLog = (log == NULL)? false : log->doLog();

	const Eigen::MatrixXd::Index Me = Ae.rows();
	const Eigen::MatrixXd::Index Mi = Ai.rows();
	const Eigen::MatrixXd::Index Msum = Me+Mi;
	const Eigen::MatrixXd::Index N = Ae.cols();

	const double deltaLin = 5.*layer;
	const double deltaAng = 1.0*Deg2Rad;
	Eigen::VectorXd deltaDiag(N);
	Eigen::VectorXd deltaInvDiag(N);
	for (Eigen::MatrixXd::Index i = 0; i < N; i++) {
		if (i%6 < 3) {
			deltaDiag[i] = deltaLin;
			deltaInvDiag[i] = 1./deltaLin;
		} else {
			deltaDiag[i] = deltaAng;
			deltaInvDiag[i] = 1./deltaAng;
		}
	}
	const Eigen::VectorXd epsilonDiag = Eigen::VectorXd::Ones(Msum)*layer;
	const Eigen::VectorXd epsilonInvDiag = Eigen::VectorXd::Ones(Msum)/layer;
	Eigen::MatrixXd B(Msum,Msum+N);
	B.block(0,0,Me,N) = Ae*deltaDiag.asDiagonal();
	B.block(Me,0,Mi,N) = Ai*deltaDiag.asDiagonal();
	Eigen::VectorXd x(N);
	Eigen::VectorXd b(Msum);
	b.block(0,0,Me,1) = be;
	b.block(Me,0,Mi,1) = bi;

	double alpha = 1;
	bool iterate = true;
	unsigned int k = 0;
	if (doLog)
		log->beginSection("Minimization Iterations",RWPE_LOCATION);
	for (; k < 10 && iterate; k++) {
		iterate = false;
		B.block(0,N,Msum,Msum) = (epsilonDiag*(-1.0/sqrt(alpha))).asDiagonal();
		const Eigen::MatrixXd Binv = LinearAlgebra::pseudoInverse(B,svdPrecision);
		const Eigen::VectorXd y = Binv*b;
		x = deltaDiag.asDiagonal()*y.topRows(N);
		const Eigen::VectorXd s = epsilonDiag.asDiagonal()*y.bottomRows(Msum)/sqrt(alpha);
		const Eigen::VectorXd se = s.topRows(Me);
		const Eigen::VectorXd si = s.bottomRows(Mi);

		if (doLog) {
			std::stringstream name;
			name << "Alpha " << alpha;
			std::ostream& lstr = log->log(name.str(),RWPE_LOCATION);
			lstr << "Equality error: " << se.transpose() << std::endl;
			lstr << "Inequality (contact max penetration) error: " << si.transpose() << std::endl;
			lstr << "Displacement: " << x.transpose() << std::endl;
		}

		for (Eigen::VectorXd::Index i = 0; i < si.rows(); i++) {
			if (s[i] > epsilonDiag[i]) {
				alpha *= 2;
				iterate = true;
				break;
			}
		}
		for (Eigen::VectorXd::Index i = 0; i < x.rows(); i++) {
			// for objects that have contacts we try to limit the displacement to avoid causing new penetrations in tight fitting scenarios
			if (hasContacts[i/6]) {
				if (x[i] > deltaDiag[i]) {
					if (iterate)
						RW_THROW("RWPEConstraintCorrection (minimize): could not correct.");
					alpha /= 2;
					iterate = true;
					break;
				}
			}
		}
	}
	if (doLog) {
		log->endSection(__LINE__);

		std::vector<std::string> labels;
		labels.push_back("Iterations");
		labels.push_back("Alpha");
		std::vector<double> values;
		values.push_back(k);
		values.push_back(alpha);
		log->addValues("Correction Minimization Result",values,labels,RWPE_LOCATION);
	}

	return x;
}

void RWPEConstraintCorrection::addDefaultProperties(PropertyMap& properties) {
	//properties.add<double>(PROPERTY_PROPTHRESHOLDFACTOR,"Angle between a pair of contact normals must be less than this factor multiplied by the distance between the contact points to be included (degrees per meter).",10.0/0.003);
	properties.add<double>(PROPERTY_PROPLAYER,"The maximum penetration allowed in a contact after correction (in meters).",0.0005);
}
