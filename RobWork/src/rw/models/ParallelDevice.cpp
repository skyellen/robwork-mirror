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


#include "ParallelDevice.hpp"
#include "ParallelLeg.hpp"
#include "Joint.hpp"

#include <rw/kinematics/FrameMap.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>

#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/models/SphericalJoint.hpp>
#include <rw/models/PrismaticSphericalJoint.hpp>
#include <rw/models/UniversalJoint.hpp>
#include <rw/models/PrismaticUniversalJoint.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

namespace
{
	void append(std::vector<Joint*>& result, const std::vector<Joint*>& tail)
	{
		result.insert(result.end(), tail.begin(), tail.end());
	}

    std::vector<Joint*> getActuatedJoints(const std::vector<ParallelLeg*>& legs)
    {
        std::vector<Joint*> result;
        for (size_t i = 0; i < legs.size(); i++)
            append(result, legs[i]->getActuatedJoints());
        return result;
    }

    std::vector<Joint*> getActuatedJoints(const std::vector<Joint*>& joints) {
        std::vector<Joint*> result;
        for (size_t i = 0; i < joints.size(); i++) {
        	if (joints[i]->isActive())
        		result.push_back(joints[i]);
        }
        return result;
    }

    std::vector<Joint*> getUnactuatedJoints(const std::vector<ParallelLeg*>& legs)
    {
        std::vector<Joint*> result;
        for (size_t i = 0; i < legs.size(); i++)
            append(result, legs[i]->getUnactuatedJoints());
        return result;
    }

    std::vector<Joint*> getUnactuatedJoints(const std::vector<Joint*>& joints) {
        std::vector<Joint*> result;
        for (size_t i = 0; i < joints.size(); i++) {
        	if (!joints[i]->isActive())
        		result.push_back(joints[i]);
        }
        return result;
    }

    std::vector<Joint*> getAllJointsFromLegs(const std::vector<ParallelLeg*>& legs) {
    	std::vector<Joint*> joints;
    	for(std::vector< ParallelLeg* >::const_iterator leg_iter = legs.begin(); leg_iter!=legs.end(); leg_iter++) {
    		const ParallelLeg* const leg = *leg_iter;
    		for(std::vector<Frame*>::const_iterator iter = leg->getKinematicChain().begin(); iter != leg->getKinematicChain().end(); iter++){
    			Joint* const joint = dynamic_cast<Joint*>(*iter);
    			if(joint == NULL)
    				continue;
    			joints.push_back(joint);
    		}
    	}
    	return joints;
    }

    int getJointDOFs(const std::vector<Joint*>& joints)
    {
        int DOFs = 0;
        for (size_t i = 0; i < joints.size(); i++)
            DOFs += joints[i]->getDOF();
        return DOFs;
    }
}

ParallelDevice::ParallelDevice(
    const std::vector<ParallelLeg*>& legs,
    const std::string name,
    const State& state)
    :
    JointDevice(
        name,
        legs.front()->getBase(),
        legs.front()->getEnd(),
        getActuatedJoints(legs),
        state),

    _legs(legs),
	_junctions(1,legs),
    _actuatedJoints(getActuatedJoints(legs)),
    _unActuatedJoints(getUnactuatedJoints(legs)),
	_joints(getAllJointsFromLegs(legs))
{
}

ParallelDevice::ParallelDevice(const std::string name, Frame* base, Frame* end, const std::vector<Joint*>& joints, const State& state, const std::vector<Legs>& junctions):
	JointDevice(name, base, end, getActuatedJoints(joints), state),
	_junctions(junctions),
	_actuatedJoints(getActuatedJoints(joints)),
	_unActuatedJoints(getUnactuatedJoints(joints)),
	_joints(joints)
{
}

ParallelDevice::~ParallelDevice() {
}

void ParallelDevice::setQ(const Q& q, State& s) const {
	const std::vector<bool> enabled(q.size(),true);
	setQ(q, enabled, s);
}

namespace {
void updateActuatedJacobian(
		const std::vector<ParallelDevice::Legs>& junctions,
		const FrameMap<Eigen::MatrixXd::Index>& qIndexActuatedAll,
		const FrameMap<Eigen::MatrixXd::Index>& qIndexActuatedEnabled,
		const std::vector<bool>& enabled,
		const State& state,
		Eigen::MatrixXd& jacobian)
{
	int row=0;
	for(std::size_t ji = 0; ji < junctions.size(); ji++) {
		const ParallelDevice::Legs& legs = junctions[ji];
		for(std::size_t i = 0; i < legs.size(); i++) {
			const Jacobian::Base leg_jacobian = legs[i]->baseJend(state).e();
			std::vector<Frame*>::const_iterator iter = legs[i]->getKinematicChain().begin();
			for(size_t j = 0; iter != legs[i]->getKinematicChain().end(); ++iter) { // the columns
				const Joint* const joint = dynamic_cast<const Joint*>(*iter);
				if( joint == NULL )
					continue;
				// determine if joint j is an active joint
				const Joint* depJoint = NULL;
				double scale = 1;
				int DOFs = joint->getDOF();
				bool active = joint->isActive();
				if (const DependentJoint* const djoint = dynamic_cast<const DependentJoint*>(*iter)) {
					if (const DependentRevoluteJoint* const drjoint = dynamic_cast<const DependentRevoluteJoint*>(djoint)) {
						depJoint = &drjoint->getOwner();
						scale = drjoint->getScale();
						DOFs = depJoint->getDOF();
						active = active && depJoint->isActive();
					} else if (const DependentPrismaticJoint* const dpjoint = dynamic_cast<const DependentPrismaticJoint*>(djoint)) {
						depJoint = &dpjoint->getOwner();
						scale = dpjoint->getScale();
						DOFs = depJoint->getDOF();
						active = active && depJoint->isActive();
					} else {
						RW_WARN("ParallelDevice only supports dependent joints if they are of revolute or prismatic type. Dependent joint is ignored.");
					}
				}
				if(active){
					// copy the leg_jacobian column at index j into the actuated jacobian matrix
					std::size_t ja_column = qIndexActuatedEnabled[*joint];
					std::size_t j_enabler = qIndexActuatedAll[*joint];
					if (depJoint != NULL) {
						ja_column = qIndexActuatedEnabled[*depJoint];
						j_enabler = qIndexActuatedAll[*depJoint];
						DOFs = depJoint->getDOF();
					}
					std::size_t cntEn = 0;
					if(i != legs.size()-1) {
						for (int d = 0; d < DOFs; d++) {
							if (enabled[j_enabler+d]) {
								jacobian.block(row,ja_column+cntEn,6,1) = leg_jacobian.col(j+d)*scale;
								cntEn++;
							}
						}
					}
					if(i!=0){
						cntEn = 0;
						for (int d = 0; d < DOFs; d++) {
							if (enabled[j_enabler+d]) {
								jacobian.block(row-6,ja_column+cntEn,6,1) = -leg_jacobian.col(j+d)*scale;
								cntEn++;
							}
						}
					}
				}
				j += DOFs;
			}
			row += 6;
		}
		row -= 6;
	}
}

void updateUnactuatedJacobian(
		const std::vector<ParallelDevice::Legs>& junctions,
		const FrameMap<Eigen::MatrixXd::Index>& qIndexActuatedAll,
		const FrameMap<Eigen::MatrixXd::Index>& qIndexActuatedEnabled,
		const FrameMap<Eigen::MatrixXd::Index>& qIndexUnactuated,
		const std::vector<bool>& enabled,
		const State& state,
		Eigen::MatrixXd& jacobian)
{
	int row = 0;
	//update Unactuated Joint Jacobian
	for(std::size_t ji = 0; ji < junctions.size(); ji++) {
		const ParallelDevice::Legs& legs = junctions[ji];
		for(std::size_t i = 0; i < legs.size(); i++) {
			const Jacobian::Base leg_jacobian = legs[i]->baseJend(state).e();
			std::vector<Frame*>::const_iterator iter = legs[i]->getKinematicChain().begin();
			for(size_t j = 0; iter != legs[i]->getKinematicChain().end(); ++iter) {
				const Joint* const joint = dynamic_cast<const Joint*>(*iter);
				if( joint == NULL )
					continue;
				const Joint* depJoint = NULL;
				double scale = 1;
				int DOFs = joint->getDOF();
				bool active = joint->isActive();
				if (const DependentJoint* const djoint = dynamic_cast<const DependentJoint*>(*iter)) {
					if (const DependentRevoluteJoint* const drjoint = dynamic_cast<const DependentRevoluteJoint*>(djoint)) {
						depJoint = &drjoint->getOwner();
						scale = drjoint->getScale();
						DOFs = depJoint->getDOF();
						active = active && depJoint->isActive();
					} else if (const DependentPrismaticJoint* const dpjoint = dynamic_cast<const DependentPrismaticJoint*>(djoint)) {
						depJoint = &dpjoint->getOwner();
						scale = dpjoint->getScale();
						DOFs = depJoint->getDOF();
						active = active && depJoint->isActive();
					} else {
						RW_WARN("ParallelDevice only supports dependent joints if they are of revolute or prismatic type. Dependent joint is ignored.");
					}
				}
				if (active) {
					std::size_t jua_column = qIndexUnactuated[*joint];
					std::size_t j_enabler = qIndexActuatedAll[*joint];
					if (depJoint != NULL) {
						jua_column = qIndexUnactuated[*depJoint];
						j_enabler = qIndexActuatedAll[*depJoint];
					}
					std::size_t cntDis = 0;
					if (i != legs.size()-1) {
						for (int d = 0; d < DOFs; d++) {
							if (!enabled[j_enabler+d]) {
								jacobian.block(row,jua_column+cntDis,6,1) = leg_jacobian.col(j+d)*scale;
								cntDis++;
							}
						}
					}
					if (i != 0) {
						cntDis = 0;
						for (int d = 0; d < DOFs; d++) {
							if (!enabled[j_enabler+d]) {
								jacobian.block(row-6,jua_column+cntDis,6,1) = -leg_jacobian.col(j+d)*scale;
								cntDis++;
							}
						}
					}
				}
				j += DOFs;
			}
			row += 6;
		}
		row -= 6;
	}
	row = 0;
	for(std::size_t ji = 0; ji < junctions.size(); ji++) {
		const ParallelDevice::Legs& legs = junctions[ji];
		for(std::size_t i = 0; i < legs.size(); i++) {
			const Jacobian::Base leg_jacobian = legs[i]->baseJend(state).e();
			std::vector<Frame*>::const_iterator iter = legs[i]->getKinematicChain().begin();
			for(size_t j = 0; iter != legs[i]->getKinematicChain().end(); ++iter) {
				const Joint* const joint = dynamic_cast<const Joint*>(*iter);
				if( joint == NULL )
					continue;
				const Joint* depJoint = NULL;
				double scale = 1;
				int DOFs = joint->getDOF();
				bool active = joint->isActive();
				if (const DependentJoint* const djoint = dynamic_cast<const DependentJoint*>(*iter)) {
					if (const DependentRevoluteJoint* const drjoint = dynamic_cast<const DependentRevoluteJoint*>(djoint)) {
						depJoint = &drjoint->getOwner();
						scale = drjoint->getScale();
						DOFs = depJoint->getDOF();
						active = active && depJoint->isActive();
					} else if (const DependentPrismaticJoint* const dpjoint = dynamic_cast<const DependentPrismaticJoint*>(djoint)) {
						depJoint = &dpjoint->getOwner();
						scale = dpjoint->getScale();
						DOFs = depJoint->getDOF();
						active = active && depJoint->isActive();
					} else {
						RW_WARN("ParallelDevice only supports dependent joints if they are of revolute or prismatic type. Dependent joint is ignored.");
					}
				}
				if(!active) {
					// copy the jacobian row into the unactuated jacobian matrix
					std::size_t jua_column = qIndexUnactuated[*joint];
					if (depJoint != NULL) {
						jua_column = qIndexUnactuated[*depJoint];
					}
					if(i != legs.size()-1){
						for (int d = 0; d < DOFs; d++) {
							jacobian.block(row,jua_column+d,6,1) = leg_jacobian.block(0,j+d,6,1)*scale;
						}
					}
					if(i!=0){
						for (int d = 0; d < DOFs; d++) {
							jacobian.block(row-6,jua_column+d,6,1) = -leg_jacobian.block(0,j+d,6,1)*scale;
						}
					}
				}
				j += DOFs;
			}
			row += 6;
		}
		row -= 6;
	}
}
}

void ParallelDevice::setQ(const Q& q, const std::vector<bool>& enabled, State& s) const {
    // default MAX_ITERATIONS.
    const int MAX_ITERATIONS = 20;

    std::size_t cntDisabled = 0;
    for (std::size_t i = 0; i < q.size(); i++) {
    	if (!enabled[i])
    		cntDisabled++;
    }

    // construct the current joint value vectors for actuated and unactuated joints
    Q lastAJVal(Q::zero(getJointDOFs(_actuatedJoints)-cntDisabled));
    Q lastUAJVal(Q::zero(getJointDOFs(_unActuatedJoints)+cntDisabled));

    // create the actuated joint jacobian, should be the sice of row = 6* number of legs
    // and columns = number of active joints
    const int aDOFs = getJointDOFs(_actuatedJoints);
    const int uaDOFs = getJointDOFs(_unActuatedJoints);
    Eigen::MatrixXd::Index nCon = 0;
    for (std::size_t i = 0; i < _junctions.size(); i++) {
    	const Legs& legs = _junctions[i];
    	if (legs.size() >= 2) {
    		nCon += legs.size()-1;
    	}
    }
    Eigen::MatrixXd aJointJ = Eigen::MatrixXd::Zero(nCon*6, aDOFs-cntDisabled); // the actuated joint jacobian
    Eigen::MatrixXd uaJointJ = Eigen::MatrixXd::Zero(nCon*6, uaDOFs+cntDisabled); // the unactuated joint jacobian

	std::size_t cur;
	cur = 0;
	{
		std::size_t cntEn = 0;
		for(size_t i=0;i<_actuatedJoints.size();i++) {
			for (int d = 0; d < _actuatedJoints[i]->size(); d++) {
				if (enabled[cur]) {
					lastAJVal(cntEn) = _actuatedJoints[i]->getData(s)[d];
					cntEn++;
				}
				cur++;
			}
		}
	}
	cur = 0;
	{
		std::size_t cntDis = 0;
		for(size_t i=0;i<_actuatedJoints.size();i++) {
			for (int d = 0; d < _actuatedJoints[i]->size(); d++) {
				if (!enabled[cur]) {
					lastUAJVal(cntDis) = _actuatedJoints[i]->getData(s)[d];
					cntDis++;
				}
				cur++;
			}
		}
		cur = cntDis;
		for(size_t i=0;i<_unActuatedJoints.size();i++) {
			for (int d = 0; d < _unActuatedJoints[i]->size(); d++) {
				lastUAJVal(cur) = _unActuatedJoints[i]->getData(s)[d];
				cur++;
			}
		}
	}

	State state(s);

    cur = 0;
    for (std::size_t i=0; i < _actuatedJoints.size(); i++) {
    	Q redQ(_actuatedJoints[i]->getDOF());
    	for (int d = 0; d < _actuatedJoints[i]->getDOF(); d++) {
    		if (enabled[cur+d])
    			redQ[d] = q[cur+d];
    		else
    			redQ[d] = _actuatedJoints[i]->getData(s)[d];
    	}
    	_actuatedJoints[i]->setData(state, &redQ[0]);
    	cur += _actuatedJoints[i]->getDOF();
    }

    if (uaDOFs+cntDisabled == 0) {
    	s = state;
    	return;
    }

    // Find the first index in the equation system for each joint
    FrameMap<Eigen::MatrixXd::Index> qIndexActuatedAll;
    FrameMap<Eigen::MatrixXd::Index> qIndexActuatedEnabled;
    FrameMap<Eigen::MatrixXd::Index> qIndexUnactuated;
    {
    	Eigen::MatrixXd::Index dAct = 0;
    	Eigen::MatrixXd::Index dUnact = 0;
    	for (std::size_t i = 0; i < _actuatedJoints.size(); i++) {
    		Eigen::MatrixXd::Index dActTmp = 0;
    		Eigen::MatrixXd::Index dUnactTmp = 0;
			qIndexActuatedAll[*_actuatedJoints[i]] = dAct+dUnact;
    		for (int d = 0; d < _actuatedJoints[i]->getDOF(); d++) {
    			if (enabled[dAct+dUnact+d]) {
    				qIndexActuatedEnabled[*_actuatedJoints[i]] = dAct;
    				dActTmp++;
    			} else {
    	    		qIndexUnactuated[*_actuatedJoints[i]] = dUnact;
    				dUnactTmp++;
    			}
    		}
    		dAct += dActTmp;
    		dUnact += dUnactTmp;
    	}
    	for (std::size_t i = 0; i < _unActuatedJoints.size(); i++) {
    		qIndexUnactuated[*_unActuatedJoints[i]] = dUnact;
    		dUnact += _unActuatedJoints[i]->getDOF();
    	}
    }

    // initialize configuration vector
	updateActuatedJacobian(_junctions, qIndexActuatedAll, qIndexActuatedEnabled, enabled, state, aJointJ);
	updateUnactuatedJacobian(_junctions, qIndexActuatedAll, qIndexActuatedEnabled, qIndexUnactuated, enabled, state, uaJointJ);

    double e = 1e-6;
    Q deltaQA(Q::zero(aDOFs-cntDisabled));
    {
		std::size_t cntEn = 0;
		for(std::size_t i=0; i < static_cast<std::size_t>(aDOFs); i++) {
			if (enabled[i]) {
				deltaQA(cntEn) = q(i) - lastAJVal(cntEn);
				cntEn++;
			}
	    }
    }

    // calculate -aJ(q)*dQa , aJ is te actuated jacobian and dQa is difference between
    // current and desired actuated joint val
    Q aJdeltaQA(-aJointJ*deltaQA.e());

    // Calculate the initial change of the unactuated joints
    Q deltaQUA(LinearAlgebra::pseudoInverse(uaJointJ)*aJdeltaQA.e());

    // Solve the equation uaJ(q)*dQua = dY , where dY = deltaPoses, for dQua
    //Q deltaY(Q::zero((int)_unActuatedJoints.size()));
    Q deltaY(Q::zero(nCon*6));

    int iterations = 0;
    double error = 1;
    //double lasterror = 1;
    do{
        // update the unactuated joints and unactuated joint jacobian
        for(std::size_t i=0; i < (std::size_t)uaDOFs+cntDisabled; i++) {
    		lastUAJVal(i) = lastUAJVal(i) + deltaQUA[i];
        }
    	{
        	std::size_t cur = 0;
    		std::size_t cntDis = 0;
    		for(std::size_t i = 0; i < _actuatedJoints.size(); i++) {
    			Q qAct(_actuatedJoints[i]->getDOF());
    			for (int d = 0; d < _actuatedJoints[i]->size(); d++) {
    				if (enabled[cur])
    					qAct[d] = _actuatedJoints[i]->getData(state)[d];
    				else {
    					qAct[d] = lastUAJVal[cntDis];
    					cntDis++;
    				}
					cur++;
    			}
    			_actuatedJoints[i]->setData(state, &qAct[0]);
    		}
    		cur = cntDis;
        	for(std::size_t i=0; i < _unActuatedJoints.size(); i++) {
                _unActuatedJoints[i]->setData(state, &lastUAJVal(cur));
                cur += _unActuatedJoints[i]->getDOF();
            }
    	}

    	updateUnactuatedJacobian(_junctions, qIndexActuatedAll, qIndexActuatedEnabled, qIndexUnactuated, enabled, state, uaJointJ);

        // update deltaY
        int row = 0;
        for(std::size_t ji = 0; ji < _junctions.size(); ji++) {
        	const Legs& legs = _junctions[ji];
        	for(std::size_t i = 0; i < legs.size()-1; i++) {
        		// Calculate quaternion and pos for i
        		Transform3D<> bTe = legs[i]->baseTend(state);
        		// Calculate quaternion and pos for i+1
        		Transform3D<> bTe_1 = legs[i+1]->baseTend(state);
        		// Calculate the difference between pose1 and pose2
        		Vector3D<> pos = bTe_1.P() - bTe.P();
        		EAA<> orin = bTe.R()*(EAA<>( inverse(bTe.R())*bTe_1.R() ) );
        		// copy it into deltaY

        		deltaY[row+0] = pos(0);
        		deltaY[row+1] = pos(1);
        		deltaY[row+2] = pos(2);

        		deltaY[row+3] = orin(0);
        		deltaY[row+4] = orin(1);
        		deltaY[row+5] = orin(2);
            	row += 6;
        	}
        }

        // Calculate the new change in the unactuated joints
        error = deltaY.norm2();
        deltaQUA = Q(LinearAlgebra::pseudoInverse(uaJointJ)*deltaY.e());
        iterations++;
    } while( e < error && iterations < MAX_ITERATIONS);

    if (iterations >= MAX_ITERATIONS){
    	RW_WARN("Could not find a valid configuration for ParallelDevice! Please check your device model.");
    } else {
       s = state;
    }

    normalizeJoints(s);
}

// Jacobians

Jacobian ParallelDevice::baseJend(const State& state) const {
    // Find the first index in the equation system for each joint
    FrameMap<Eigen::MatrixXd::Index> qIndex;
    std::size_t columns = 0;
    {
    	Eigen::MatrixXd::Index d = 0;
    	for (std::size_t i = 0; i < _joints.size(); i++) {
    		qIndex[*_joints[i]] = d;
			d += _joints[i]->getDOF();
    	}
    	columns = d;
    }

    Eigen::MatrixXd::Index rows = 0;
    for (std::size_t i = 0; i < _junctions.size(); i++) {
    	const Legs& legs = _junctions[i];
    	if (legs.size() >= 2) {
    		rows += (legs.size()-1)*6;
    	}
    }
    rows += _junctions.size()*6;

	Jacobian::Base m = Jacobian::zero(rows, columns).e();

    int row = 0;
    for(std::size_t ji = 0; ji < _junctions.size(); ji++) {
    	const Legs& legs = _junctions[ji];
    	for(std::size_t i = 0; i < legs.size(); i++) {
    		const Jacobian::Base leg_jacobian = legs[i]->baseJend(state).e();
    		std::vector<Frame*>::const_iterator iter = legs[i]->getKinematicChain().begin();
    		for(size_t j = 0; iter != legs[i]->getKinematicChain().end(); ++iter) {
    			const Joint* joint = dynamic_cast<const Joint*>(*iter);
    			if(joint == NULL)
    				continue;
				const Joint* depJoint = NULL;
				double scale = 1;
				int DOFs = joint->getDOF();
				if (const DependentJoint* const djoint = dynamic_cast<const DependentJoint*>(*iter)) {
					if (const DependentRevoluteJoint* const drjoint = dynamic_cast<const DependentRevoluteJoint*>(djoint)) {
						depJoint = &drjoint->getOwner();
						scale = drjoint->getScale();
						DOFs = depJoint->getDOF();
					} else if (const DependentPrismaticJoint* const dpjoint = dynamic_cast<const DependentPrismaticJoint*>(djoint)) {
						depJoint = &dpjoint->getOwner();
						scale = dpjoint->getScale();
						DOFs = depJoint->getDOF();
					} else {
						RW_WARN("ParallelDevice only supports dependent joints if they are of revolute or prismatic type. Dependent joint is ignored.");
					}
				}
    			// copy the jacobian row into the jacobian matrix
    			std::size_t j_column = qIndex[*joint];
				if (depJoint != NULL) {
					j_column = qIndex[*depJoint];
				}
    			if(i != legs.size()-1) {
					m.block(row,j_column,6,DOFs) = leg_jacobian.block(0,j,6,DOFs)*scale;
    			}
    			if(i != 0) {
					m.block(row-6,j_column,6,DOFs) = -leg_jacobian.block(0,j,6,DOFs)*scale;
    			}
    			j += DOFs;
    		}
    		row += 6;
    	}
    	row -= 6;
    }

	return Jacobian( m );
}

Jacobian ParallelDevice::baseJend(const std::vector<ParallelLeg*>& legs, const State& state) {
    // calculate the size of the total jacobian matrix and configuration vector
    size_t rows=0, columns=0;
    std::vector< ParallelLeg* >::const_iterator leg_iter = legs.begin();
    for(;leg_iter!=legs.end();++leg_iter){
        rows += 6;
        columns += (*leg_iter)->getJointDOFs();
    }


    Jacobian::Base m = Jacobian::zero(rows, columns).e();

    // initialize configuration vector
    int row=0,column=0;
    for(size_t i=0;i<legs.size();i++){
        // copy the base to endeffector jacobian of leg i to the jacobian matrix at index (row,column)

		m.block(row,column,6,legs[i]->getJointDOFs()) = legs[i]->baseJend(state).e();

        if(i!=0){ // for all legs except the first copy -bTe jacobian of leg into (row-6,column)
    		m.block(row-6,column,6,legs[i]->getJointDOFs()) = -legs[i]->baseJend(state).e();
        }

        // update the row and column count
        row += 6; // allways for robots in 3d
        column += (int)legs[i]->getJointDOFs();
    }

    return Jacobian( m );
}

Jacobian ParallelDevice::baseJframe(const Frame* frame, const State& state) const
{
    RW_THROW("Not implemented");

    // calculate the size of the total jacobian matrix and configuration vector
    size_t rows = 0, columns = 0;
    std::vector<ParallelLeg*>::const_iterator leg_iter = _legs.begin();
    for(;leg_iter!=_legs.end();++leg_iter){
        rows += 6;
        columns += (*leg_iter)->getJointDOFs();
    }

    return Jacobian::zero(rows,columns);
}

std::vector<Joint*> ParallelDevice::getAllJoints() const {
	return _joints;
}

std::size_t ParallelDevice::getFullDOF() const {
	std::size_t dof = 0;
    //for(std::vector< ParallelLeg* >::const_iterator leg_iter = _legs.begin(); leg_iter!=_legs.end(); leg_iter++) {
    //    dof += (*leg_iter)->getJointDOFs();
    //}
	for (std::size_t i = 0; i < _joints.size(); i++)
		dof += _joints[i]->getDOF();
    return dof;
}

std::pair<Q, Q> ParallelDevice::getAllBounds() const {
    const Q q(getFullDOF());
    std::pair<Q, Q> bounds(q, q);
    int i = 0;
    /*
    for(std::vector< ParallelLeg* >::const_iterator leg_iter = _legs.begin(); leg_iter!=_legs.end(); leg_iter++) {
    	const ParallelLeg* const leg = *leg_iter;
    	for(std::vector<Frame*>::const_iterator iter = leg->getKinematicChain().begin(); iter != leg->getKinematicChain().end(); iter++){
    		const Joint* const joint = dynamic_cast<const Joint*>(*iter);
    		if(joint == NULL)
    			continue;
            const std::pair<Q, Q> pair = joint->getBounds();
            bounds.first.setSubPart(i,pair.first);
            bounds.second.setSubPart(i, pair.second);
            i += joint->getDOF();
    	}
    }
    */
	for (std::size_t ji = 0; ji < _joints.size(); ji++) {
        const std::pair<Q, Q> pair = _joints[ji]->getBounds();
        bounds.first.setSubPart(i,pair.first);
        bounds.second.setSubPart(i, pair.second);
        i += _joints[ji]->getDOF();
	}
    return bounds;
}

Q ParallelDevice::getFullQ(const State& state) const {
	Q q(getFullDOF());
	std::size_t cur = 0;
	/*
    for(std::vector< ParallelLeg* >::const_iterator leg_iter = _legs.begin(); leg_iter!=_legs.end(); leg_iter++) {
    	const ParallelLeg* const leg = *leg_iter;
    	for(std::vector<Frame*>::const_iterator iter = leg->getKinematicChain().begin(); iter != leg->getKinematicChain().end(); iter++){
    		const Joint* const joint = dynamic_cast<const Joint*>(*iter);
    		if(joint == NULL)
    			continue;
    		for (int d = 0; d < joint->size(); d++) {
    			q[cur] = joint->getData(state)[d];
    			cur++;
    		}
    	}
    }
    */
	for (std::size_t i = 0; i < _joints.size(); i++) {
		for (int d = 0; d < _joints[i]->size(); d++) {
			q[cur] = _joints[i]->getData(state)[d];
			cur++;
		}
	}
    return q;
}

void ParallelDevice::setFullQ(const Q& q, State& state) const {
	std::size_t cur = 0;
	/*
    for(std::vector< ParallelLeg* >::const_iterator leg_iter = _legs.begin(); leg_iter!=_legs.end(); leg_iter++) {
    	const ParallelLeg* const leg = *leg_iter;
    	for(std::vector<Frame*>::const_iterator iter = leg->getKinematicChain().begin(); iter != leg->getKinematicChain().end(); iter++){
    		const Joint* const joint = dynamic_cast<const Joint*>(*iter);
    		if(joint == NULL)
    			continue;
			joint->setData(state,&q[cur]);
			cur += joint->size();
    	}
    }
    */
	for (std::size_t i = 0; i < _joints.size(); i++) {
		_joints[i]->setData(state,&q[cur]);
		cur += _joints[i]->size();
	}
    normalizeJoints(state);
}

void ParallelDevice::normalizeJoints(State& state) const {
	/*
    for(std::vector< ParallelLeg* >::const_iterator leg_iter = _legs.begin(); leg_iter!=_legs.end(); leg_iter++) {
    	const ParallelLeg* const leg = *leg_iter;
    	for(std::vector<Frame*>::const_iterator iter = leg->getKinematicChain().begin(); iter != leg->getKinematicChain().end(); iter++){
    		const Joint* const joint = dynamic_cast<const Joint*>(*iter);
    		if(joint == NULL)
    			continue;
			Q q(joint->size());
			const std::pair<Q,Q> bounds = joint->getBounds();
    		for (int d = 0; d < joint->size(); d++) {
    			q[d] = joint->getData(state)[d];
    		}
    		if (dynamic_cast<const SphericalJoint*>(joint) || dynamic_cast<const PrismaticSphericalJoint*>(joint)) {
    			for (std::size_t i = 0; i < 3; i++) {
    				while(q[i] > bounds.second[i])
    					q[i] -= 2*Pi;
    				while(q[i] < bounds.first[i])
    					q[i] += 2*Pi;
    			}
    		} else if (dynamic_cast<const UniversalJoint*>(joint) || dynamic_cast<const PrismaticUniversalJoint*>(joint)) {
    			for (std::size_t i = 0; i < 2; i++) {
    				while(q[i] > bounds.second[i])
    					q[i] -= 2*Pi;
    				while(q[i] < bounds.first[i])
    					q[i] += 2*Pi;
    			}
    		}
			joint->setData(state,&q[0]);
    	}
    }
    */
	for (std::size_t i = 0; i < _joints.size(); i++) {
		Joint* const joint = _joints[i];
		Q q(joint->size());
		const std::pair<Q,Q> bounds = joint->getBounds();
		for (int d = 0; d < joint->size(); d++) {
			q[d] = joint->getData(state)[d];
		}
		if (dynamic_cast<const SphericalJoint*>(joint) || dynamic_cast<const PrismaticSphericalJoint*>(joint)) {
			for (std::size_t i = 0; i < 3; i++) {
				while(q[i] > bounds.second[i])
					q[i] -= 2*Pi;
				while(q[i] < bounds.first[i])
					q[i] += 2*Pi;
			}
		} else if (dynamic_cast<const UniversalJoint*>(joint) || dynamic_cast<const PrismaticUniversalJoint*>(joint)) {
			for (std::size_t i = 0; i < 2; i++) {
				while(q[i] > bounds.second[i])
					q[i] -= 2*Pi;
				while(q[i] < bounds.first[i])
					q[i] += 2*Pi;
			}
		}
		joint->setData(state,&q[0]);
	}
}
