// QtoPos.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <fstream>

#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;

int main(int argc, char* argv[])
{
    if (argc <= 2) {
        std::cout<<"QtoPos <Input> <Output>"<<std::endl;
        return -1;
    }

    std::auto_ptr<WorkCell> workcell = XMLRWLoader::LoadWorkCell(
        "d:/workspace/RobWorkData/XMLScenes/MobileManipulator/MobileManipulator.xml");

    Device* device;
    for (std::vector<Device*>::const_iterator it =
             workcell->getDevices().begin();
         it != workcell->getDevices().end();
         ++it)
    {
        if ((*it)->getName() == "KukaLBR3")  {
            device = (*it);
        }
    }

    if (device == NULL) {
        RW_WARN("Error: No KukaLBR3 Device Found");
        return -1;
    }

    Frame* gripper = workcell->findFrame("PG70.TCP");
    State state = workcell->getDefaultState();

    FKRange fkrange(device->getBase(), gripper, state);
    std::ifstream in(argv[1]);
    std::ofstream out(argv[2]);
    while (!in.eof()) {
        Q q(7);
        double t;
        in>>t;
        for (int i = 0; i<7; i++)
            in>>q(i);
        device->setQ(q, state);
        Transform3D<> transform = fkrange.get(state);
        out<<t<<" "<<transform.P()(0)<<" "<<transform.P()(1)<<" "<<transform.P()(2)<<std::endl;
    }
    in.close();
    out.close();

    return 0;
}
