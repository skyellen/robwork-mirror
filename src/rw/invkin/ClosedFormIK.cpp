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

#include "ClosedFormIK.hpp"
#include "PieperSolver.hpp"

#include <rw/models/JointDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::invkin;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::math;

ClosedFormIKPtr ClosedFormIK::make(const Device& device,
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
    std::vector<DHSet> dhs;
    for (size_t i = 0; i < jd->getDOF(); i++) {
        Joint* joint = jd->getActiveJoint(i);

        RevoluteJoint* rj = dynamic_cast<RevoluteJoint*>(joint);
        if (!rj)
            RW_THROW(
                "Joint " << *joint << " of device "
                << device << " is not revolute.");

        DHSet* dh = Accessor::dhSet().getPtr(*joint);
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
    const Transform3D<> lastToEnd = Kinematics::frameTframe(jd->getActiveJoint(jd->getDOF() - 1),
                                                            device.getEnd(),
                                                            state);

    return ownedPtr(new PieperSolver(dhs, lastToEnd));
}
