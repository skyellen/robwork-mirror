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


#include "ClosedFormIK.hpp"
#include "PieperSolver.hpp"

#include <rw/models/JointDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/DHParameterSet.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <boost/foreach.hpp>

using namespace rw::invkin;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::math;

ClosedFormIK::Ptr ClosedFormIK::make(const Device& device,
                                   const State& state)
{
    // Cast the device.
    const JointDevice* jd = dynamic_cast<const JointDevice*>(&device);

    if (!jd)
        RW_THROW("Device " << device << " is not a subtype of JointDevice.");

    // Check DOFs.
    if (jd->getDOF()) {
        RW_THROW(
            "Device " << device << " is not a 6 DOF device. DOF is "
            << jd->getDOF());
    }

    // Extract the DH parameters.
    std::vector<DHParameterSet> dhs;
    Joint *lastJoint = NULL;
    BOOST_FOREACH(Joint *joint, jd->getJoints() ){
    //for (size_t i = 0; i < jd->getDOF(); i++) {
    //    Joint* joint = jd->getActiveJoint(i);
        lastJoint = joint;
        RevoluteJoint* rj = dynamic_cast<RevoluteJoint*>(joint);
        if (!rj)
            RW_THROW(
                "Joint " << *joint << " of device "
                << device << " is not revolute.");

        const DHParameterSet* dh = DHParameterSet::get(joint);
        if (!dh) {
            RW_THROW(
                "No Denavit-Hartenberg parameters for joint "
                << *joint
                << " of device "
                << device);
        } else {
            dhs.push_back(*dh);
        }
    }

    // Find the transform from the last joint to the end of the device.
    const Transform3D<> lastToEnd = Kinematics::frameTframe(lastJoint,
                                                            device.getEnd(),
                                                            state);

    return ownedPtr(new PieperSolver(dhs, lastToEnd));
}
