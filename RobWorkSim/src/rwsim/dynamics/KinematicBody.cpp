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

#include "KinematicBody.hpp"

#include <rw/models/Joint.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <rw/kinematics/MovableFrame.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;

using namespace rwsim::dynamics;

KinematicBody::KinematicBody(
            const BodyInfo& info,
            rw::models::Object::Ptr obj):
			   Body(6, info, obj),
			   _base(NULL)

{
    _base = dynamic_cast<MovableFrame*>(obj->getBase());
    if(_base==NULL){
        RW_THROW("Base frame of Object in a KinematicBody must be a MovableFrame!");
    }
}

KinematicBody::~KinematicBody()
{
}

void KinematicBody::reset(rw::kinematics::State &state){
	// set variables in state to 0
	Q zeroVec = Q::zero(6);
	double *vel = this->getData(state);
	for(size_t i=0;i<6;i++){
		vel[i] = 0;
	}
}

rw::math::VelocityScrew6D<> KinematicBody::getVelocity(const rw::kinematics::State &state) const{
    return rw::math::VelocityScrew6D<>();
}
