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

#include "MovableFrame.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Quaternion.hpp>

using namespace rw::kinematics;
using namespace rw::math;

MovableFrame::MovableFrame(
    Frame* parent,
    const std::string& name)
    :
    //Frame(parent, 6, name)
    Frame(parent, 7, name)
{}

Transform3D<> MovableFrame::getTransform(const State& state) const
{
    const double* q = getQ(state);
    //const RPY<> rpy(q[0], q[1], q[2]);
    //const Vector3D<> pos(q[3], q[4], q[5]);
    Quaternion<> quat(q[0],q[1],q[2],q[3]);
    const Vector3D<> pos(q[4], q[5], q[6]);
    quat.normalize();
   
    return Transform3D<>(pos, quat);
}

void MovableFrame::setTransform(const Transform3D<>& transform, State& state)
{
    //const RPY<> rpy(transform.R());
    const Quaternion<> quat(transform.R());
    const Vector3D<> pos(transform.P());

    /*double q[6];
    q[0] = rpy(0);
    q[1] = rpy(1);
    q[2] = rpy(2);
    q[3] = pos(0);
    q[4] = pos(1);
    q[5] = pos(2);
*/
    double q[7];
    q[0] = quat(0);
    q[1] = quat(1);
    q[2] = quat(2);
    q[3] = quat(3);
    q[4] = pos(0);
    q[5] = pos(1);
    q[6] = pos(2);
    setQ(state, q);
}
