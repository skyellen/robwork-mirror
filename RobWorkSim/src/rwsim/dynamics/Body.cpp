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

#include "Body.hpp"

#include <boost/numeric/ublas/matrix.hpp>

#include <rw/math/LinearAlgebra.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace boost::numeric;

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rwsim::dynamics;


rw::math::Vector3D<> Body::getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const {
    Transform3D<> wTp = Kinematics::worldTframe(getParentFrame(state), state);
    Transform3D<> wTb = Kinematics::worldTframe(getBodyFrame(), state);

    VelocityScrew6D<> vel = getVelocity(state);
    // first transform point to body frame
    rw::math::Vector3D<> posOnBody = inverse(wTp).R() * (p - wTb.P());
    // then calculate the velocity of the point relative to the body frame
    EAA<> e = vel.angular();
    Vector3D<> tmp(e[0],e[1],e[2]);
    rw::math::Vector3D<> pVelBody = vel.linear() + cross(tmp, posOnBody);
    // adn last remember to transform velocity back to world frame
    return wTp.R() * pVelBody;
}
