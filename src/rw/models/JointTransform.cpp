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

#include "JointTransform.hpp"

#include <rw/math/EAA.hpp>

using namespace rw::models;
using namespace rw::math;

Transform3D<> JointTransform::getRevoluteTransform(
    const Transform3D<>& displacement, double q)
{
    return
        displacement *
        Transform3D<>(
            Vector3D<>(0, 0, 0),
            EAA<>(0, 0, q).toRotation3D());
}

Transform3D<> JointTransform::getPrismaticTransform(
    const Transform3D<>& displacement, double q)
{
    Transform3D<> move = Transform3D<>::Identity();
    move(2,3) = q;
    return displacement * move;
}
