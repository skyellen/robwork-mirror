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


#include "IterativeIK.hpp"
#include "JacobianIKSolver.hpp"

#include <rw/common/macros.hpp>
#include <rw/models/Device.hpp>

using namespace rw::invkin;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace boost;

IterativeIK::IterativeIK()
{
    getProperties().add(
        "MaxIterations", "Max number of iterations", 20);
    getProperties().add(
        "MaxError", "Max Error ", 1e-6);
}

void IterativeIK::setMaxError(double maxError)
{
    if (maxError < 0)
        RW_THROW("MaxError must be positive");

    getProperties().set<double>("MaxError", maxError);
}

double IterativeIK::getMaxError() const
{
    return getProperties().get<double>("MaxError");
}

void IterativeIK::setMaxIterations(int maxIterations)
{
    getProperties().set("MaxIterations", maxIterations);
}

int IterativeIK::getMaxIterations() const
{
    return getProperties().get<int>("MaxIterations");
}

PropertyMap& IterativeIK::getProperties()
{
    return _properties;
}

const PropertyMap& IterativeIK::getProperties() const
{
    return _properties;
}

IterativeIK::Ptr IterativeIK::makeDefault(Device::Ptr device, const State& state)
{
    return ownedPtr(new JacobianIKSolver(device, state));
}
