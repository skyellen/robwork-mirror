#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <string>
#include <boost/foreach.hpp>

void printKinematicTree(
    Frame& frame,
    const State& state,
    const Transform3D<>& parentTransform,
    int level)
{
    const Transform3D<> transform = parentTransform * frame.getTransform(state);

    std::cout
        << std::string(level, ' ')
        << frame.getName()
        << " at "
        << transform.P()
        << "\n";

    BOOST_FOREACH(Frame& child, frame.getChildren(state)) {
        printKinematicTree(child, state, transform, level + 1);
    }
}

void printDefaultWorkCellStructure(const WorkCell& workcell)
{
    std::cout << workcell << "\n";
    printKinematicTree(
        *workcell.getWorldFrame(),
        workcell.getDefaultState(),
        Transform3D<>::Identity(),
        0);
}
