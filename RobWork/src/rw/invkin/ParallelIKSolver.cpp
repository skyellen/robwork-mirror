/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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

#include "ParallelIKSolver.hpp"

#include <rw/common/macros.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/math/Jacobian.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/VectorND.hpp>

#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/Models.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/ParallelLeg.hpp>

#include <vector>

using rw::common::ownedPtr;
using namespace rw::invkin;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

ParallelIKSolver::ParallelIKSolver(const ParallelDevice* device):
    _device(device),
	_junctions(_device->getJunctions()),
    _useJointClamping(false),
    _checkJointLimits(false)
{
	Eigen::MatrixXd::Index d = 0;
	const std::vector<Joint*> joints = device->getAllJoints();
	for (std::size_t i = 0; i < joints.size(); i++) {
		_jointIndex[*joints[i]] = d;
		d += joints[i]->getDOF();
	}
}

ParallelIKSolver::~ParallelIKSolver() {
}

std::vector<Q> ParallelIKSolver::solve(const Transform3D<>& dest, const State& wstate) const {
	return solve(std::vector<Target>(1,Target(_device->getEnd(),dest)),wstate);
}

void ParallelIKSolver::updateDeltaX(const std::vector<Target>& targets, const FramePairMap<ParallelLeg::Ptr>& targetLegs, const State& state, Q& deltaX, const Eigen::MatrixXd::Index nCon) const {
	std::size_t index = nCon;
	for (std::size_t ti = 0; ti < targets.size(); ti++) {
		const Target& target = targets[ti];
		const Frame* const targetRefFrame = (targets[ti].refFrame == NULL)? _device->getBase() : targets[ti].refFrame;
		if (!targetLegs.has(targetRefFrame, target.tcpFrame))
			RW_THROW("The frame " << target.tcpFrame->getName() << " is not valid as end-effector in ParallelIKSolver! The frame must lie in one of the legs.");
		const ParallelLeg::Ptr tleg = targetLegs(targetRefFrame, target.tcpFrame);
		const Transform3D<>& T = target.refTtcp;

		// calculate the difference from current pose to dest pose
		const Transform3D<double> curr = Kinematics::frameTframe(targetRefFrame,target.tcpFrame,state);
		const Vector3D<> pos = T.P() - curr.P();
		const EAA<> orin = curr.R()*(EAA<>( inverse(curr.R())*T.R() ) );

		// init deltaX
		for (std::size_t i = 0; i < 3; i++) {
			if (target.enabled[i]) {
				deltaX[index] = pos[i];
				index++;
			}
		}
		for (std::size_t i = 3; i < 6; i++) {
			if (target.enabled[i]) {
				deltaX[index] = orin[i-3];
				index++;
			}
		}
	}
}

void ParallelIKSolver::updateJacobian(const std::vector<Target>& targets, const FramePairMap<ParallelLeg::Ptr>& targetLegs, const State& state, Jacobian& jacobian) const {
    std::size_t conDOF = 0;
    for (std::size_t i = 0; i < targets.size(); i++) {
    	conDOF += targets[i].dof();
    }

    jacobian = _device->baseJend(state);
	std::size_t row = jacobian.size1()-6*_junctions.size();
	Eigen::MatrixXd newJac = Eigen::MatrixXd::Zero(row+conDOF,jacobian.size2());
	newJac.block(0,0,row,jacobian.size2()) = jacobian.e().block(0,0,row,jacobian.size2());

	for (std::size_t ti = 0; ti < targets.size(); ti++) {
		const Target& target = targets[ti];
		const Frame* const targetRefFrame = (targets[ti].refFrame == NULL)? _device->getBase() : targets[ti].refFrame;
		if (!targetLegs.has(targetRefFrame, target.tcpFrame))
			RW_THROW("The frame " << target.tcpFrame->getName() << " is not valid as end-effector in ParallelIKSolver! The frame must lie in one of the legs.");
		const ParallelLeg::Ptr tleg = targetLegs(targetRefFrame, target.tcpFrame);

		const Jacobian& serialJac = tleg->baseJframe(target.tcpFrame,state);
		const std::vector<Frame*> frames = tleg->getKinematicChain();
		int dof = 0;
		for (std::size_t frameI = 0; frameI < frames.size(); frameI++) {
			const Joint* const joint = dynamic_cast<const Joint*>(frames[frameI]);
			if (joint == NULL)
				continue;

			const Joint* depJoint = NULL;
			double scale = 1;
			int DOFs = joint->getDOF();
			if (const DependentJoint* const djoint = dynamic_cast<const DependentJoint*>(joint)) {
				if (const DependentRevoluteJoint* const drjoint = dynamic_cast<const DependentRevoluteJoint*>(djoint)) {
					depJoint = &drjoint->getOwner();
					scale = drjoint->getScale();
					DOFs = depJoint->getDOF();
				} else if (const DependentPrismaticJoint* const dpjoint = dynamic_cast<const DependentPrismaticJoint*>(djoint)) {
					depJoint = &dpjoint->getOwner();
					scale = dpjoint->getScale();
					DOFs = depJoint->getDOF();
				} else {
					RW_WARN("ParallelIKSolver only supports dependent joints if they are of revolute or prismatic type. Dependent joint is ignored.");
				}
			}
			std::size_t tmpRow = row;
			for (std::size_t i = 0; i < 6; i++) {
				if (target.enabled[i]) {
					Eigen::MatrixXd::Index index = _jointIndex[*joint];
					if (depJoint != NULL) {
						index = _jointIndex[*depJoint];
					}
					newJac.block(tmpRow,index,1,DOFs) = serialJac.e().block(i,dof,1,DOFs)*scale;
					tmpRow++;
				}
			}
			dof += DOFs;
		}
		row += target.dof();
	}
	jacobian = Jacobian(newJac);
}

std::vector<Q> ParallelIKSolver::solve(const std::vector<Target>& targets, const State& wstate) const {
    State state = wstate;
    const double maxerror = getMaxError();
    const int maxiterations = getMaxIterations();

    const Device::QBox bounds = _device->getAllBounds();

    FramePairMap<ParallelLeg::Ptr> targetLegs;
	const std::vector<Joint*> allJoints = _device->getAllJoints();
    for(std::size_t i = 0; i < targets.size(); i++) {
		Frame* refFrame = NULL;
		const Frame* const targetRefFrame = (targets[i].refFrame == NULL)? _device->getBase() : targets[i].refFrame;
		for (std::size_t i = 0; i < _junctions.size(); i++) {
			const std::vector<ParallelLeg*>& legs = _junctions[i];
			for (std::size_t legI = 0; legI < legs.size(); legI++) {
				const ParallelLeg* const leg = legs[legI];
				const std::vector<Frame*>& chain = leg->getKinematicChain();
				Frame* searchFrame = chain.back();
				while (searchFrame != NULL && refFrame == NULL) {
					if (searchFrame == targetRefFrame) {
						refFrame = searchFrame;
					}
					searchFrame = searchFrame->getParent(state);
				}
			}
		}
		if (refFrame == NULL)
			RW_THROW("Could not find reference frame " << targetRefFrame->getName() << " in parallel device!");

		Frame* tcpFrame = NULL;
		const Frame* frame = targets[i].tcpFrame;
		while(frame != NULL && tcpFrame == NULL) {
			for (std::size_t i = 0; i < allJoints.size(); i++) {
				if (allJoints[i] == frame)
					tcpFrame = allJoints[i];
			}
			if (tcpFrame == NULL)
				frame = frame->getParent(state);
		}
        if (tcpFrame == NULL)
        	RW_THROW("Could not find end frame " << targets[i].tcpFrame->getName() << " in parallel device!");

    	if (!targetLegs.has(targetRefFrame, targets[i].tcpFrame)) {
    		std::vector<Frame*> chain = Kinematics::parentToChildChain(refFrame, tcpFrame, wstate);
    		chain.push_back(tcpFrame);
    		if (chain.size() == 0)
    			RW_THROW("Could not find connection from reference frame " << targetRefFrame->getName() << " to end frame " << targets[i].tcpFrame->getName() << " in parallel device!");
    		targetLegs(targetRefFrame, targets[i].tcpFrame) = ownedPtr(new ParallelLeg(chain));
    	}
    }

    std::size_t conDOF = 0;
    for (std::size_t i = 0; i < targets.size(); i++) {
    	conDOF += targets[i].dof();
    }

    // initialize currQ
    const std::size_t DOFs = _device->getFullDOF();
    Q currQ = _device->getFullQ(state);

    // initialize deltaQ
    Q deltaQ(Q::zero(DOFs));

    // initialize deltaX vector
    Eigen::MatrixXd::Index nCon = 0;
    for (std::size_t i = 0; i < _junctions.size(); i++) {
    	const std::vector<ParallelLeg*>& legs = _junctions[i];
    	if (legs.size() >= 2) {
    		nCon += legs.size()-1;
    	}
    }
    Q deltaX = Q::zero(nCon*6+conDOF);

    updateDeltaX(targets, targetLegs, state, deltaX, nCon*6);

    Jacobian jacobian(0,0);
    updateJacobian(targets, targetLegs, state, jacobian);

    int iterations=0;
    double error=1;

    const std::vector<Joint*> joints = _device->getAllJoints();
    do {
    	deltaQ = Q(LinearAlgebra::pseudoInverse(jacobian.e())*deltaX.e());

    	// update the state of the joints with deltaQ
    	currQ += deltaQ;
    	if(_useJointClamping && iterations < maxiterations-1)
    		currQ = Math::clampQ(currQ, bounds.first, bounds.second);
        std::size_t qIndex = 0;
    	for (std::size_t i = 0; i < joints.size(); i++) {
    		joints[i]->setData(state, &currQ(qIndex));
    		qIndex += joints[i]->getDOF();
    	}

    	// update deltaX
		std::size_t row = 0;
    	for(std::size_t ji = 0; ji < _junctions.size(); ji++) {
    		const std::vector<ParallelLeg*>& legs = _junctions[ji];
    		for(size_t i = 0; i < legs.size()-1; i++) {
    			// Calculate quaternion and pos for i
    			Transform3D<double> bTe = legs[i]->baseTend(state);
    			// Calculate quaternion and pos for i+1
    			Transform3D<double> bTe_1 = legs[i+1]->baseTend(state);
    			// Calculate the difference between pose1 and pose2
    			Vector3D<double> pos = bTe_1.P() - bTe.P();
    			EAA<double> orin = bTe.R()*(EAA<>( inverse(bTe.R())*bTe_1.R() ) );

    			// copy it into deltaX
    			deltaX[row+0] = pos(0);
    			deltaX[row+1] = pos(1);
    			deltaX[row+2] = pos(2);
    			deltaX[row+3] = orin(0);
    			deltaX[row+4] = orin(1);
    			deltaX[row+5] = orin(2);
    			row += 6;
    		}
    	}

    	updateDeltaX(targets, targetLegs, state, deltaX, nCon*6);
    	updateJacobian(targets, targetLegs, state, jacobian);

    	error = deltaX.norm2();
    	//last_error = error;
    	iterations++;
    } while( maxerror < error && iterations < maxiterations);

    if (iterations >= maxiterations && maxerror < error){
    	std::cout << "ERROR: ParallelIKSolver max iterations exeeded!!! Max error is " << deltaX << std::endl;
    	std::cout << "Jacobian dimensions: " << jacobian.size1() << "x" << jacobian.size2() << std::endl;
    }

    Q q = _device->getQ(state);
    std::vector<Q> sol;
    if (!_checkJointLimits) {
    	sol.push_back(q);
    } else if (Models::inBounds(_device->getFullQ(state), bounds)) {
    	sol.push_back(q);
    }

    return sol;
}

void ParallelIKSolver::setCheckJointLimits(bool check) {
	_checkJointLimits = check;
}

void ParallelIKSolver::setClampToBounds(bool enableClamping) {
	_useJointClamping = enableClamping;
}
