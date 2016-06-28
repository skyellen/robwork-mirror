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

#include "AmbiguityResolver.hpp"

#include <rw/models/JointDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>

#include <boost/foreach.hpp>

using namespace rw::invkin;
using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;
    
AmbiguityResolver::AmbiguityResolver(const InvKinSolver::Ptr& invkin, rw::models::JointDevice::Ptr device):
    _invkin(invkin),
    _device(device)
{
    _invkin->setCheckJointLimits(false);
    
    const std::vector<Joint*>& joints = device->getJoints();
    size_t index = 0;
    BOOST_FOREACH(const class Joint* joint, joints) {
        if (dynamic_cast<RevoluteJoint*>(joints[index])) {
            for (int i = 0; i<joint->getDOF(); i++) {
                _indices.push_back(index);
                ++index;
            } 
        } else {
            index += joint->getDOF();
        }
    }
    _lower = device->getBounds().first;
    _upper = device->getBounds().second;
}


AmbiguityResolver::~AmbiguityResolver(void) {

}

std::vector<Q> AmbiguityResolver::solve(const Transform3D<>& baseTend, const State& state) const {
    std::vector<Q> res1 = _invkin->solve(baseTend, state);
    std::vector<Q> res2; 
    const double pi2 = 2*math::Pi;
    
    
    BOOST_FOREACH(size_t index, _indices) { 
        res2.clear();
//        std::cout<<"Index = "<<index<<std::endl;
        BOOST_FOREACH(const Q& q, res1) {    
            //std::cout<<"Input= "<<q<<std::endl;

            double d = q(index);
            while (d>_lower(index))
                d -= pi2;
            while (d<_lower(index))
                d += pi2;
            while (d <_upper(index)) {
                Q tmp(q);
                tmp(index) = d;
                res2.push_back(tmp);
              //  std::cout<<"Output = "<<tmp<<std::endl;
                d += pi2;
            }
        }
        res1 = res2;
        //std::cout<<"Output Count = "<<res1.size()<<std::endl;
    }
    return res1;
}

void AmbiguityResolver::setCheckJointLimits(bool check) {
    
}
