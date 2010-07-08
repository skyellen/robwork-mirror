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

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;

using namespace rwsim::dynamics;

KinematicBody::KinematicBody(
            const BodyInfo& info,
            rw::kinematics::Frame& frame,
            const std::vector<GeometryPtr>& geoms,
            rw::kinematics::State& state):
			   Body(6, info, &frame, geoms),
			   _base(&frame)

{
}

KinematicBody::~KinematicBody()
{
}

void KinematicBody::rollBack(State &state){
	// no aux variables
}

void KinematicBody::saveState(double h, rw::kinematics::State& state){
	// no aux variables
}

void KinematicBody::resetState(rw::kinematics::State &state){
	// set variables in state to 0
	Q zeroVec = Q::zero(6);
	double *vel = this->getQ(state);
	for(size_t i=0;i<6;i++){
		vel[i] = 0;
	}
}

rw::math::Vector3D<> KinematicBody::getPointVelW(const rw::math::Vector3D<>& wPb, const rw::kinematics::State& state) const{
	// we need a state to make this calculation
	// TODO: velocity is expressed in parent coordinates and this should therefore also
	Transform3D<> wTb = Kinematics::worldTframe(_base, state);
    // first transform point to body frame
    rw::math::Vector3D<> posOnBody = (wPb - wTb.P());
    // then calculate the velocity of the point relative to the body frame
    rw::math::Vector3D<> pVelBody = getLinVel(state) + cross(getAngVel(state), posOnBody);
    // adn last remember to transform velocity back to world frame
    return wTb.R() * pVelBody;
}
