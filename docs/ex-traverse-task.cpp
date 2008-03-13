#include <rw/task/Task.hpp>
#include <rw/task/Trajectory.hpp>
#include <rw/task/Action.hpp>
#include <rw/common/PropertyMap.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <boost/foreach.hpp>

void visitPropertyMap(const std::string& indent, const PropertyMap& propertyMap)
{
    if (!propertyMap.empty()) std::cout << indent << "Properties: ";

    BOOST_FOREACH(PropertyBase* prop, propertyMap.getProperties()) {
        std::cout
            << prop->getIdentifier()
            << " ";
    }

    if (!propertyMap.empty()) std::cout << "\n";
}

void visitTarget(const Target& target)
{
    std::cout
        << "    Target "
        << target.getName()
        << "\n";

    visitPropertyMap("    ", target.getPropertyMap());

    if (const Q* to = boost::get<Q>(&target.getValue())) {
        std::cout << "      Move device to Q of DOF " << to->size() << "\n";
    }

    else if (
        const ToolLocation* location = boost::get<ToolLocation>(&target.getValue()))
    {
        std::cout
            << "      Move tool to "
            << location->getTransform().P()
            << " relative to "
            << location->getFrame()
            << "\n";
    }
}

void visitTrajectory(const Trajectory& trajectory)
{
    std::cout
        << "  Trajectory "
        << trajectory.getName()
        << "\n";

    visitPropertyMap("  ", trajectory.getPropertyMap());

    BOOST_FOREACH(const Target& target, trajectory.getTargets()) {
        visitTarget(target);
    }
}

void visitAction(const Action& action)
{
    const AttachFrameAction& attach =
        boost::get<AttachFrameAction>(action.getValue());

    std::cout
        << "  Attach "
        << attach.getChild().getName()
        << " to "
        << attach.getParent().getName()
        << "\n";

    visitPropertyMap("  ", action.getPropertyMap());
}

void visitTask(const Task& task)
{
    std::cout << "Task " << task.getName() << "\n";

    visitPropertyMap("", task.getPropertyMap());

    BOOST_FOREACH(const Task::TaskElement& element, task) {
        if (const Trajectory* trajectory = boost::get<Trajectory>(&element)) {
            visitTrajectory(*trajectory);
        }
        else if (const Action* action = boost::get<Action>(&element)) {
            visitAction(*action);
        }
    }
}
