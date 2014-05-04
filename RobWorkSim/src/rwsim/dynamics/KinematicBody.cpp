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

#include <rw/math/Q.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/Joint.hpp>


using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;

using namespace rwsim::dynamics;

KinematicBody::KinematicBody(
            const BodyInfo& info,
            rw::models::Object::Ptr obj):
			   Body(info, obj),
			   _base(NULL)

{
    _base = dynamic_cast<MovableFrame*>(obj->getBase());
    if(_base==NULL){
        RW_THROW("Base frame of Object in a KinematicBody must be a MovableFrame!");
    }
    //std::cout << "Adding state in kinematicbody!!!!" << std::endl;
    add(_kstate);
}

KinematicBody::~KinematicBody()
{
}

void KinematicBody::reset(rw::kinematics::State &state){
	// set variables in state to 0
    KinematicBodyState &ks = _kstate.get(state);
    ks.linvel = Vector3D<>(0,0,0);
    ks.angvel = Vector3D<>(0,0,0);
}

rw::math::VelocityScrew6D<> KinematicBody::getVelocity(const rw::kinematics::State &state) const{
    const KinematicBodyState &ks = _kstate.get(state);
    return rw::math::VelocityScrew6D<>(ks.linvel[0], ks.linvel[1], ks.linvel[2],
                                        ks.angvel[0], ks.angvel[1], ks.angvel[2]);
}
