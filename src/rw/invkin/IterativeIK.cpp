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

#include "IterativeIK.hpp"
#include "ResolvedRateSolver.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/shared_ptr.hpp>

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

IterativeIKPtr IterativeIK::makeDefault(
    DevicePtr device,
    const State& state)
{
    return ownedPtr(new ResolvedRateSolver(device, state));
}
