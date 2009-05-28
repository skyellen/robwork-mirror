/*********************************************************************
 * RobWork Version 0.3
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

#include "Models.hpp"
#include "CompositeDevice.hpp"
#include <rw/kinematics/Kinematics.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::trajectory;



std::vector<Frame*> Models::findAllFrames(
    const WorkCell& workcell)
{
    return Kinematics::findAllFrames(
        workcell.getWorldFrame(),
        workcell.getDefaultState());
}

Frame& Models::getFrame(const WorkCell& workcell, const std::string& name)
{
    Frame* frame = workcell.findFrame(name);
    if (!frame)
        RW_THROW(
            "No frame named "
            << StringUtil::quote(name)
            << " in workcell "
            << workcell);
    return *frame;
}

Device& Models::getDevice(const WorkCell& workcell, const std::string& name)
{
    Device* device = workcell.findDevice(name);
    if (!device)
        RW_THROW(
            "No device named "
            << StringUtil::quote(name)
            << " in workcell "
            << workcell);
    return *device;
}

namespace
{
    bool inOrder(double a, double b, double c, double tolerance)
    {
        return a - tolerance < b &&  b < c + tolerance;
    }
}

bool Models::inBounds(const Q& val,
                      const Joint& joint,
                      double tolerance)
{
    return inBounds(val, joint.getBounds(), tolerance);
    /*for (size_t i = 0; i<val.size(); i++)
        if (!inOrder(joint.getBounds().first(i), val(i), joint.getBounds().second(i), tolerance))
            return false;
    return true;*/
}

bool Models::inBounds(const Q& q,
                      const Device::QBox& bounds,
                      double tolerance)
{
    RW_ASSERT(tolerance >= 0);

    const Q& a = bounds.first;
    const Q& b = bounds.second;

    const int len = a.size();
    for (int i = 0; i < len; i++) {
        const double val = q[i];
        if (!inOrder(a[i], val, b[i], tolerance))
            return false;
    }
    return true;
}

bool Models::inBounds(const Q& q, const Device& device, double tolerance)
{
    return inBounds(q, device.getBounds(), tolerance);
}

bool Models::inBounds(
    const State& state,
    const WorkCell& workcell,
    double tolerance)
{
    const std::vector<Frame*>& frames = findAllFrames(workcell);

    typedef std::vector<Frame*>::const_iterator I;
    for (I p = frames.begin(); p != frames.end(); ++p) {
        const Joint* joint = dynamic_cast<const Joint*>(*p);
        if (joint) {
            const Q val = Q(joint->getDOF(),joint->getQ(state));
            if (!inBounds(val, *joint, tolerance))
                return false;
        }
    }

    return true;
}

void Models::getStatePath(
    const Device& device,
    const QPath& path,
    const State& common_state,
    StatePath& result)
{
    State state = common_state;
    BOOST_FOREACH(const Q& q, path) {
        device.setQ(q, state);
        result.push_back(state);
    }
}

StatePath Models::getStatePath(
    const Device& device,
    const QPath& path,
    const State& common_state)
{
    StatePath result;
    getStatePath(device, path, common_state, result);
    return result;
}

rw::models::DevicePtr Models::makeDevice(
    rw::models::DevicePtr device,
    const State& state,
    rw::kinematics::Frame* base,
    rw::kinematics::Frame* end)
{
    RW_ASSERT(device);

    // Set the defaults.
    if (!base) base = device->getBase();
    if (!end) end = device->getEnd();

    // Wrap the device if base or end differs.
    if (base != device->getBase() || end != device->getEnd())
        return ownedPtr(
            new CompositeDevice(
                base,
                std::vector<DevicePtr>(1, device),
                end,
                "Models::makeDevice(" + device->getName() + ")",
                state));
    else
        return device;
}
