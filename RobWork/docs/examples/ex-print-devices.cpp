#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

using namespace rw::models;

#include <boost/foreach.hpp>

void printDeviceNames(const WorkCell& workcell)
{
    std::cout << "Workcell " << workcell << " contains devices:\n";
    BOOST_FOREACH(Device* device, workcell.getDevices()) {
        std::cout << "- " << device->getName() << "\n";
    }
}
