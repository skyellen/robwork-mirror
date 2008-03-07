/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#include "TaskLoader.hpp"

#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/task/Trajectory.hpp>
#include <rw/task/Task.hpp>

typedef boost::property_tree::ptree PTree;

using namespace std;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::task;
using namespace boost::property_tree;

namespace
{
    string quote(const string& str) { return StringUtil::Quote(str); }

    // This is just a utility that is useful when figuring out how property
    // trees are structured. Please let it stay here even though it isn't being
    // called in production code.
    void printTree(const PTree& tree, int level)
    {
        string indent(2 * level, ' ');

        if (tree.size() == 0) {
            std::cout << indent << "Leaf: '" << tree.get_own<string>() << "'\n";
        }
        else {
            for (PTree::const_iterator p = tree.begin(); p != tree.end(); ++p) {
                std::cout
                    << indent << p->first << "\n"
                    << indent << "{\n";
                printTree(p->second, level + 1);
                std::cout
                    << indent << "}\n";
            }
        }
    }

    typedef PTree::const_iterator CI;

    Q readNArray(const PTree& tree)
    {
        Q q(tree.size());
        int i = 0;
        for (CI p = tree.begin(); p != tree.end(); ++p, ++i) {

            if (p->first != "N") {
                RW_THROW(
                    "Unexpected XML tag "
                    << quote(p->first)
                    << " where numbers <N> were expected.");
            }

            q[i] = p->second.get_own<double>();
        }
        return q;
    }

    Vector3D<> readVector3D(const PTree& tree)
    {
        const Q vals = readNArray(tree);
        if (vals.size() != 3)
            RW_THROW(
                "Unexpected number of values "
                << (int)vals.size()
                << " for Vector3D.");

        return Vector3D<>(vals[0], vals[1], vals[2]);
    }

    Rotation3D<> readRotation3D(const PTree& tree)
    {
        if (tree.find("Rotation3D") != tree.end()) {
            const Q vals = readNArray(tree.get_child("Rotation3D"));
            if (vals.size() != 9) {
                RW_THROW(
                    "Unexpected number of values "
                    << (int)vals.size()
                    << " for Rotation3D.");
            }

            return Rotation3D<>(
                vals[0], vals[1], vals[2],
                vals[3], vals[4], vals[5],
                vals[6], vals[7], vals[8]);

        } else if (tree.find("RPY") != tree.end()) {
            const Q vals = readNArray(tree.get_child("RPY"));
            if (vals.size() != 3)
                RW_THROW(
                    "Unexpected number of RPY values "
                    << (int)vals.size());

            return RPY<>(vals[0], vals[1], vals[2]).toRotation3D();
        } else {
            RW_THROW("No rotation specified. <Rotation3D> or <RPY> expected.");
        }
    }

    Transform3D<> readTransform3D(const PTree& tree)
    {
        return Transform3D<>(
            readVector3D(tree.get_child("Vector3D")),
            readRotation3D(tree));
    }

    Q readQ(const PTree& tree)
    {
        return readNArray(tree.get_child("Q"));
    }

    ToolSpeed readToolSpeed(const PTree& tree)
    {
        const double speed = tree.get<double>("ToolSpeed.N");
        if (tree.get_child_optional("ToolSpeed.Angular")) {
            return ToolSpeed(ToolSpeed::Angular, speed);
        } else if (tree.get_child_optional("ToolSpeed.Positional")) {
            return ToolSpeed(ToolSpeed::Positional, speed);
        } else {
            RW_THROW(
                "No tool speed specified. <Angular> or <Positional> expected.");
            return readToolSpeed(tree);
        }
    }

    string getOptionalName(const PTree& tree)
    {
        return tree.get<string>("Name", "");
    }

    Link readLink(const PTree& tree)
    {
        const string name = getOptionalName(tree);

        if (tree.find("LinearToolConstraint") != tree.end()) {
            return Link(
                LinearToolConstraint(
                    readToolSpeed(tree.get_child("LinearToolConstraint"))),
                name);
        } else if (tree.find("LinearJointConstraint") != tree.end()) {
            return Link(LinearJointConstraint(), name);
        } else {
            return Link(NoConstraint(), name);
        }
    }

    ToolLocation readToolLocation(const PTree& tree, const WorkCell& workcell)
    {
        const string frame_name = tree.get<string>("Frame");
        Frame* frame = workcell.findFrame(frame_name);
        if (!frame)
            RW_THROW(
                "Reference frame "
                << quote(frame_name)
                << " not found in workcell "
                << workcell);

        return ToolLocation(
            readTransform3D(tree.get_child("Transform3D")),
            frame);
    }

    Target readTarget(
        const PTree& tree, const WorkCell& workcell, const Device& device)
    {
        const string name = getOptionalName(tree);

        if (tree.find("Tool") != tree.end()) {
            return Target(readToolLocation(tree.get_child("Tool"), workcell), name);
        } else if (tree.find("Joint") != tree.end()) {
            const Q q = readQ(tree.get_child("Joint"));

            if (q.size() != device.getDOF())
                RW_THROW(
                    "Configuration "
                    << q
                    << " for device "
                    << device
                    << " should have been of dimension "
                    << (int)device.getDOF());

            // Here we should sanity check that q has the right dimension.
            return Target(q, name);
        } else {
            RW_THROW("No target specified. <Tool> or <Joint> expected.");

            return readTarget(tree, workcell, device);
        }
    }

    Frame* readTCP(const PTree& tree, Device* device, const WorkCell& workcell)
    {
        boost::optional<string> tcp_name = tree.get_optional<string>("TCP");
        if (!tcp_name) {
            return device->getEnd();
        } else {
            Frame* tcp = workcell.findFrame(*tcp_name);

            if (!tcp)
                RW_THROW(
                    "TCP frame "
                    << quote(*tcp_name)
                    << " not found in workcell "
                    << workcell);

            return tcp;
        }
    }

    Trajectory readTrajectory(const PTree& tree, WorkCell* workcell)
    {
        const string device_name = tree.get<string>("Device");
        Device* device = workcell->findDevice(device_name);
        if (!device)
            RW_THROW(
                "No device named "
                << quote(device_name)
                << " in workcell "
                << *workcell);

        Frame* tcp = readTCP(tree, device, *workcell);

        const string name = getOptionalName(tree);

        Trajectory traj(workcell, device, tcp, name);

        // Now read the targets.
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Target") {
                traj.addTarget(
                    readTarget(p->second, *workcell, *device));
            } else if (p->first == "Link") {
                traj.addLink(
                    readLink(p->second));
            } else if (p->first != "Device" || p->first != "TCP") {
                // Nothing to do.
            } else {
                RW_THROW(
                    "Unexpected XML tag "
                    << quote(p->first)
                    << ". <Target>, <Link>, <Device>, or <TCP> expected.");
            }
        }

        return traj;
    }

    Action readAction(const PTree& tree, const WorkCell& workcell)
    {
        const string name = getOptionalName(tree);

        if (tree.find("AttachFrame") != tree.end()) {
            const string item_name = tree.get<string>("AttachFrame.Item");
            const string tcp_name = tree.get<string>("AttachFrame.TCP");

            MovableFrame* item =
                dynamic_cast<MovableFrame*>(
                    workcell.findFrame(item_name));

            if (!item) {
                RW_THROW(
                    "No movable frame named "
                    << quote(item_name)
                    << " in workcell "
                    << workcell);
            }

            Frame* tcp = workcell.findFrame(tcp_name);
            if (!tcp)
                RW_THROW(
                    "No TCP frame named "
                    << quote(tcp_name)
                    << " in workcell "
                    << workcell);

            return Action(AttachFrameAction(item, tcp), name);

        } else {
            return Action(NoAction(), name);
        }
    }

    Task getInitialTask(const PTree& tree, WorkCell* optional_workcell)
    {
        const string name = getOptionalName(tree);

        if (optional_workcell) {
            return Task(optional_workcell, name);
        } else {
            const string workcell_name = tree.get<string>("WorkCell");
            return Task(WorkCellLoader::load(workcell_name), name);
        }
    }

    Task readTask(const PTree& tree, WorkCell* optional_workcell)
    {
        Task task = getInitialTask(tree, optional_workcell);
        WorkCell& workcell = task.getWorkCell();

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Trajectory") {
                task.addTaskElement(
                    readTrajectory(p->second, &workcell));
            } else if (p->first == "Action") {
                task.addTaskElement(
                    readAction(p->second, workcell));
            }
        }

        // There is a bug in the Task library or Boost or above or ... so that
        // the below crashes if you do the more natural:
        //   return task;
        return Task(task);
    }
}

Task TaskLoader::load(const string& file, WorkCell* optional_workcell)
{
    try {
        PTree tree;
        read_xml(file, tree);

        // printTree(tree, 0);

        return readTask(tree.get_child("Task"), optional_workcell);

    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }

    // To avoid a compiler warning.
    RW_ASSERT(!"Impossible");
    return TaskLoader::load(file, optional_workcell);
}
