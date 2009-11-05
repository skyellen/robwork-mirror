#include <rw/task/Task.hpp>
#include <rw/task/Trajectory.hpp>
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

    if (const Q* to = boost::get<Q>(&target.getLocation())) {
        std::cout << "      Move device to Q of DOF " << to->size() << "\n";
    }

    else if (
        const ToolLocation* location = boost::get<ToolLocation>(&target.getLocation()))
    {
        std::cout
            << "      Move tool to "
            << location->getTransform().P()
            << " relative to "
            << location->getFrame()
            << "\n";
    }
}

void visitLink(const Link& link)
{
    std::cout
        << "    Link "
        << link.getName()
        << "\n";
}

void visitTrajectory(const Trajectory& trajectory)
{
    std::cout
        << "  Trajectory "
        << trajectory.getName()
        << "\n";

    visitPropertyMap("  ", trajectory.getPropertyMap());

    BOOST_FOREACH(const Trajectory::value_type& element, trajectory.getElements()) {
        if (const Link* link = boost::get<Link>(&element)) {
            visitLink(*link);
        }

        else if (const Target* target = boost::get<Target>(&element)) {
            visitTarget(*target);
        }
    }
}

void visitAttachFrame(const AttachFrame& attach)
{
    std::cout
        << "  Attach "
        << attach.getChild().getName()
        << " to "
        << attach.getParent().getName()
        << "\n";

    visitPropertyMap("  ", attach.getPropertyMap());
}

void visitTask(const Task& task)
{
    std::cout << "Task " << task.getName() << "\n";

    visitPropertyMap("", task.getPropertyMap());

    BOOST_FOREACH(const Task::value_type& action, task.getActions()) {
        if (const Trajectory* trajectory = boost::get<Trajectory>(&action)) {
            visitTrajectory(*trajectory);
        }
        else if (const AttachFrame* attach = boost::get<AttachFrame>(&action)) {
            visitAttachFrame(*attach);
        }
    }
}
