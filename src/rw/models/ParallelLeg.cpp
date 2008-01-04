/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "ParallelLeg.hpp"

#include <rw/common/macros.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Jacobian.hpp>

#include <rw/models/Accessor.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/kinematics/FrameProperty.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <vector>

using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::common;
using namespace boost::numeric;

using namespace rw::models;

ParallelLeg::ParallelLeg(std::vector<Frame*> frames):
    _kinematicChain(frames)
{
    std::vector<Frame*>::iterator iter = _kinematicChain.begin();
    for(;iter!=_kinematicChain.end();++iter){
        Joint *joint = dynamic_cast<Joint*>(*iter);
        if(joint!=NULL){
            if (Accessor::ActiveJoint().has(*joint) ){
                _actuatedJoints.push_back(joint);
            } else {
                _unactuatedJoints.push_back(joint);
            }
        }
    }
    _jacobian =
        new Jacobian(
            ublas::zero_matrix<double>(
                6,
                _actuatedJoints.size()+_unactuatedJoints.size()));
};

ParallelLeg::~ParallelLeg(){
    //delete _jacobian;
};

const Jacobian& ParallelLeg::baseJend(const State& state){

    // Find end-effector transform
    const Transform3D<>& vbTe = baseTend(state);
    Transform3D<> bTl = Transform3D<>::Identity();

    size_t index = 0;
    // Iterate throgh all frames
    std::vector<Frame*>::iterator iter = _kinematicChain.begin();
    for(; iter!=_kinematicChain.end(); ++iter){
        bTl = bTl * (*iter)->getTransform(state);
        // if Frame is not a joint then continue
        if( dynamic_cast<Joint*>(*iter) == NULL )
            continue;

        // extract the z vector
        Vector3D<double> z(bTl(0,2), bTl(1,2), bTl(2,2));

        if( dynamic_cast<RevoluteJoint*>(*iter) ) {
            math::Vector3D<double> p = cross(z, vbTe.P() - bTl.P());
            (*_jacobian)(0, index) = p[0];
            (*_jacobian)(1, index) = p[1];
            (*_jacobian)(2, index) = p[2];
            (*_jacobian)(3, index) = z[0];
            (*_jacobian)(4, index) = z[1];
            (*_jacobian)(5, index) = z[2];
        } else if( dynamic_cast<PrismaticJoint*>(*iter) ) {
            (*_jacobian)(0, index) = z[0];
            (*_jacobian)(1, index) = z[1];
            (*_jacobian)(2, index) = z[2];
            (*_jacobian)(3, index) = 0.0;
            (*_jacobian)(4, index) = 0.0;
            (*_jacobian)(5, index) = 0.0;
        }else{
            RW_WARN("Unknown joint type.");
        }
        index++;
    }
    return *_jacobian;
}

void ParallelLeg::setQ(const Q& q, State& state) const
{
    //RW_ASSERT( q.size() == nrOfJoints() );
    
    std::vector<Frame*>::const_iterator iter = _kinematicChain.begin();
    for(size_t i=0; iter!=_kinematicChain.end(); ++iter){
        // if Frame is not a joint then continue
        Frame *f = (Frame*) *iter;
        if( dynamic_cast<Joint*>(f) == NULL )
            continue;

        f->setQ(state, &q[i]);
        i++;
    }
}

Transform3D<double> ParallelLeg::baseTend(const State& state){
    return Kinematics::FrameTframe(_kinematicChain.front(),_kinematicChain.back(), state);
};

const std::vector<Frame*>& ParallelLeg::getKinematicChain(){
    return _kinematicChain;
}

Frame* ParallelLeg::getBase(){
    return _kinematicChain.front();
}

Frame* ParallelLeg::getEnd() {
    return _kinematicChain.back();
}

size_t ParallelLeg::nrOfActiveJoints(){
    return _actuatedJoints.size();
}

size_t ParallelLeg::nrOfPassiveJoints(){
    return _unactuatedJoints.size();
}
