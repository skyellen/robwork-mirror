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

#include "ParallelLeg.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/models/Joint.hpp>

#include <vector>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

ParallelLeg::ParallelLeg(std::vector<Frame*> frames):
    _kinematicChain(frames)
{
    std::vector<Frame*>::iterator iter = _kinematicChain.begin();
    for(;iter!=_kinematicChain.end();++iter){
        Joint *joint = dynamic_cast<Joint*>(*iter);
        if(joint!=NULL){
            if (joint->isActive() ){
                _actuatedJoints.push_back(joint);
            } else {
                _unactuatedJoints.push_back(joint);
            }
        }
    }
    _jacobian = new Jacobian(Jacobian::zero(6, getJointDOFs()).e());
}

ParallelLeg::~ParallelLeg()
{
    delete _jacobian;
}

const Jacobian& ParallelLeg::baseJend(const State& state){
    // Find end-effector transform
    const Transform3D<>& vbTe = baseTend(state);
    Transform3D<> bTl = Transform3D<>::identity();

    size_t index = 0;
    // Iterate through all frames
    std::vector<Frame*>::iterator iter = _kinematicChain.begin();
    for(iter++; iter!=_kinematicChain.end(); ++iter){
        bTl = bTl * (*iter)->getTransform(state);
        if (const Joint* const joint = dynamic_cast<const Joint*>(*iter)) {
        	for (std::size_t i = 0; i < 6; i++) {
        		for (int j = 0; j < joint->getDOF(); j++) {
        			(*_jacobian)(i, index+j) = 0;
        		}
        	}
        	joint->getJacobian(0,index,bTl,vbTe,state,*_jacobian);
        	index += joint->getDOF();
        }
    }
    return *_jacobian;
}

Q ParallelLeg::getQ(const State& state) const {
	Q q(getJointDOFs());
    std::vector<Frame*>::const_iterator iter;
    std::size_t k = 0;
    for(iter = _kinematicChain.begin(); iter!=_kinematicChain.end(); ++iter){
        // if Frame is not a joint then continue
        Frame *f = (Frame*) *iter;
        if(Joint* joint = dynamic_cast<Joint*>(f)) {
        	for (int d = 0; d < joint->getDOF(); d++) {
        		q[k] = f->getData(state)[d];
        		k++;
        	}
        }
    }
    return q;
}

void ParallelLeg::setQ(const Q& q, State& state) const
{
    std::vector<Frame*>::const_iterator iter;
    std::size_t k = 0;
    for(iter = _kinematicChain.begin(); iter!=_kinematicChain.end(); ++iter){
        // if Frame is not a joint then continue
        Frame *f = (Frame*) *iter;
        if(Joint* joint = dynamic_cast<Joint*>(f)) {
    		f->setData(state, &q[k]);
    		k += joint->getDOF();
        }
    }
}

Transform3D<double> ParallelLeg::baseTend(const State& state) const {
    return Kinematics::frameTframe(
        _kinematicChain.front(),
        _kinematicChain.back(),
        state);
}

const std::vector<Frame*>& ParallelLeg::getKinematicChain() const {
    return _kinematicChain;
}

Frame* ParallelLeg::getBase()
{
    return _kinematicChain.front();
}

Frame* ParallelLeg::getEnd()
{
    return _kinematicChain.back();
}

size_t ParallelLeg::nrOfActiveJoints()
{
    return _actuatedJoints.size();
}

size_t ParallelLeg::nrOfPassiveJoints()
{
    return _unactuatedJoints.size();
}

std::size_t ParallelLeg::getJointDOFs() const {
    int actuatedDOF = 0;
    int unactuatedDOF = 0;
    for (std::size_t i = 0; i < _actuatedJoints.size(); i++) {
    	actuatedDOF += _actuatedJoints[i]->getDOF();
    }
    for (std::size_t i = 0; i < _unactuatedJoints.size(); i++) {
    	unactuatedDOF += _unactuatedJoints[i]->getDOF();
    }
    return actuatedDOF+unactuatedDOF;
}
