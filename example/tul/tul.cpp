#include <rw/models/WorkCell.hpp>
#include <rw/models/DeviceModel.hpp>
#include <rw/models/SerialDevice.hpp>

#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

#include <iostream>

using namespace rw::models;
using namespace rw::kinematics;

int main(int argc, char** argv)
{
    if(argc != 2){
        std::cout << "Usage: tul <filename>" << std::endl;
        return -1;
    }

    std::auto_ptr<WorkCell> workcell;

    try{
        workcell = rw::loaders::WorkCellLoader::load(argv[1]);
    }catch(std::string& ex){
        std::cerr << ex << std::endl;
        exit(1);
    }

    State state = workcell->getDefaultState();
    std::cout
        << "Success: "
        << workcell->getDevices().size()
        << " devices loaded.\n";

    for (size_t i = 0; i < workcell->getDevices().size(); i++) {
        SerialDevice* device = dynamic_cast<SerialDevice*>(
            workcell->getDevices()[i]);

        if (device) {
            std::cout << device->baseTend(state) << "\n";
        }
    }

    return 0;
}
