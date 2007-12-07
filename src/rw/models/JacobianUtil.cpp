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
#include "JacobianUtil.hpp"

#include "BasicDevice.hpp"
#include "RevoluteJoint.hpp"
#include "PrismaticJoint.hpp"
#include "PassiveRevoluteFrame.hpp"

#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/FKTable.hpp>

#include <algorithm>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

using namespace boost::numeric::ublas;

typedef matrix_range<Jacobian::Base> Range;
typedef zero_matrix<double> ZeroMatrix;

namespace
{
    Vector3D<> getCol(const Rotation3D<>& rot, int col)
    {
        return Vector3D<>(
            rot(0, col),
            rot(1, col),
            rot(2, col));
    }

    void addPosition(
        Jacobian& jacobian,
        int row,
        int col,
        const Vector3D<>& pos)
    {
        int r = row*6;
        jacobian(r+0, col) += pos[0];
        jacobian(r+1, col) += pos[1];
        jacobian(r+2, col) += pos[2];
    }

    void addRotation(
        Jacobian& jacobian,
        int row,
        int col,
        const Vector3D<>& rot)
    {
        int r = row*6;
        jacobian(r+3, col) += rot[0];
        jacobian(r+4, col) += rot[1];
        jacobian(r+5, col) += rot[2];
    }
}

void JacobianUtil::addRevoluteJacobianCol(
    Jacobian& jacobian,
    int row,
    int col,
    const Transform3D<>& joint,
    const Transform3D<>& tcp)
{
    const Vector3D<> axis = getCol(joint.R(), 2);
    const Vector3D<> p = cross(axis, tcp.P() - joint.P());

    addPosition(jacobian, row, col, p);
    addRotation(jacobian, row, col, axis);
}

void JacobianUtil::addPassiveRevoluteJacobianCol(
    Jacobian& jacobian,
    int row,
    int col,
    const Transform3D<>& joint,
    const Transform3D<>& tcp,
    double scale)
{
    const Vector3D<> axis = scale * getCol(joint.R(), 2);
    const Vector3D<> p = scale * cross(axis, tcp.P() - joint.P());

    addPosition(jacobian, row, col, p);
    addRotation(jacobian, row, col, axis);
}

void JacobianUtil::addPrismaticJacobianCol(
    Jacobian& jacobian,
    int row,
    int col,
    const Transform3D<>& joint,
    const Transform3D<>& tcp)
{
    const Vector3D<> axis = getCol(joint.R(), 2);

    addPosition(jacobian, row, col, axis);
    addRotation(jacobian, row, col, Vector3D<>(0, 0, 0));
}

bool JacobianUtil::isInSubTree(
    const Frame& parent,
    const Frame& child,
    const State& state)
{
    if (&parent == &child)
        return true;
    else {
        const Frame::const_iterator_pair pair = parent.getChildren(state);
        typedef Frame::const_iterator I;
        for (I p = pair.first; p != pair.second; ++p)
            if (isInSubTree(*p, child, state))
                return true;
        return false;
    }
}

bool JacobianUtil::isControlledBy(
    const BasicDevice& device,
    const PassiveRevoluteFrame& child,
    const State& state)
{
    typedef BasicDevice::const_iterator CI;
    for (CI p = device.begin(); p != device.end(); ++p) {
        if (&*p == &child.getOwner())
            return true;
    }
    return false;
}
