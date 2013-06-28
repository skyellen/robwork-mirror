#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

USE_ROBWORK_NAMESPACE
USE_ROBWORKSIM_NAMESPACE
using namespace robwork;
using namespace robworksim;


int main(int argc, char** argv) {
	// lets assume path to test is correct
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( "../../test/testfiles/ur_control_test_scene/cup_pg70_table.dwc.xml");

    // create the simulator instance
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );

    // initialize the physics
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();
    odesim->initPhysics(state);

    // get handles of robot and robot controller to perform test
    SerialDeviceController::Ptr devctrl = dwc->findController<SerialDeviceController>("URController");
    Device::Ptr ur = dwc->getWorkcell()->findDevice("UR-6-85-5-A");

    Q target(6,0,-0.2,0,0,0,0);
    devctrl->movePTP( target, 100);
    // take 200 timesteps and observe the robot moving
    for(int i=0; i<200; i++){
    	odesim->step(0.01, state);
    	std::cout << i << ":" << ur->getQ(state) << std::endl;
    }
    return 0;
}
