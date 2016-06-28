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


#include "CompositeDevice.hpp"
#include "JacobianCalculator.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <boost/foreach.hpp>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

namespace
{

	std::vector<Joint*> concatDevices(const std::vector<Device::Ptr>& devices)
    {
        std::vector<Joint*> joints;
		BOOST_FOREACH(const Device::Ptr& ptr, devices) {
            RW_ASSERT(ptr);

            JointDevice* device = dynamic_cast<JointDevice*>(ptr.get());
            if (device) {
                const std::vector<Joint*> js = device->getJoints();
                //RW_WARN( "Nr Joints: " << js.size() );
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

	std::vector<Frame*> endFrames(const std::vector<Device::Ptr>& devices)
    {
        std::vector<Frame*> result;
		BOOST_FOREACH(const Device::Ptr& device, devices) {
            RW_ASSERT(device);
            result.push_back(device->getEnd());
        }
        return result;
    }
}

CompositeDevice::CompositeDevice(
    Frame* base,
	const std::vector<Device::Ptr>& devices,
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
	const std::vector<Device::Ptr>& devices,
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
    //return inverse(start.R()) * _djmulti->get(fk);
    return inverse(start.R()) * _djmulti->get(state);
}
