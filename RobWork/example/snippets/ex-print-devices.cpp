#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <boost/foreach.hpp>

using namespace rw::models;

void printDeviceNames(const WorkCell& workcell)
{
    std::cout << "Workcell " << workcell << " contains devices:\n";
    BOOST_FOREACH(Device::Ptr device, workcell.getDevices()) {
        std::cout << "- " << device->getName() << "\n";
    }
}
