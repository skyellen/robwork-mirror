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


#include "CollisionToleranceStrategy.hpp"

using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::math;


CollisionToleranceStrategy::CollisionToleranceStrategy() {}
CollisionToleranceStrategy::~CollisionToleranceStrategy() {}

bool CollisionToleranceStrategy::inCollision(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb,
    double tolerance)
{
    return collides(getModel(a),wTa,getModel(b),wTb,tolerance);
}
