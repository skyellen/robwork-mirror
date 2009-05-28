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

#include "CompositeDevice.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

namespace
{
    std::vector<Joint*> getJoints(JointDevice& device)
    {
        std::vector<Joint*> joints;
        const int len = device.getDOF();
        for (int i = 0; i < len; i++)
            joints.push_back(device.getActiveJoint(i));
        return joints;
    }

    std::vector<Joint*> concatDevices(const std::vector<DevicePtr>& devices)
    {
        std::vector<Joint*> joints;
        BOOST_FOREACH(const DevicePtr& ptr, devices) {
            RW_ASSERT(ptr);

            JointDevice* device = dynamic_cast<JointDevice*>(ptr.get());
            if (device) {
                const std::vector<Joint*> js = getJoints(*device);
                joints.insert(joints.end(), js.begin(), js.end());
            } else {
                // We can maybe lessen this requirement if we want by adding
                // some other facility for retrieving the sequence of active
                // joints.
                RW_THROW(
                    "CompositeDevice can't be constructed from device "
                    << *ptr
                    << " that is not of type JointDevice.");
            }
        }

        return joints;
    }

    std::vector<Frame*> endFrames(const std::vector<DevicePtr>& devices)
    {
        std::vector<Frame*> result;
        BOOST_FOREACH(const DevicePtr& device, devices) {
            RW_ASSERT(device);
            result.push_back(device->getEnd());
        }
        return result;
    }
}

CompositeDevice::CompositeDevice(
    Frame* base,
    const std::vector<DevicePtr>& devices,
    Frame* end,
    const std::string& name,
    const State& state)
    :
    JointDevice(name, base, end, concatDevices(devices), state),
    _devices(devices),
    _ends(endFrames(devices)),
    _djmulti(baseJCframes(_ends, state))
{}

CompositeDevice::CompositeDevice(
    Frame* base,
    const std::vector<DevicePtr>& devices,
    const std::vector<Frame*>& ends,
    const std::string& name,
    const State& state)
    :
    JointDevice(name, base, ends.front(), concatDevices(devices), state),
    _devices(devices),
    _ends(ends),
    _djmulti(baseJCframes(_ends, state))
{}


void CompositeDevice::setQ(const Q& q, State& state) const
{
    size_t offset = 0;
    for (size_t i = 0; i < _devices.size(); i++){

        const size_t dof = _devices[i]->getDOF();

        // This allocation of a configuration qdev is pretty slow here, but there
        // isn't any way of avoiding it currently. The setQ() procedure would
        // have to be more primitive taking for example a double* as parameter.
        Q qdev(dof);
        for (size_t j = 0; j < dof; j++)
            qdev(j) = q(offset + j);

        _devices[i]->setQ(qdev, state);

        offset += dof;
    }
}

Jacobian CompositeDevice::baseJends(const State& state) const
{
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _djmulti->get(fk);
}
