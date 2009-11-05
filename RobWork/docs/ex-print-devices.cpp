#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

#include <boost/foreach.hpp>

void printDeviceNames(const WorkCell& workcell)
{
    std::cout << "Workcell " << workcell << " contains devices:\n";
    BOOST_FOREACH(Device* device, workcell.getDevices()) {
        std::cout << "- " << device->getName() << "\n";
    }
}
