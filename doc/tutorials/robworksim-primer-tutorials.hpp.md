RobWorkSim - Dynamic Simulation in RobWork  {#pageRobWorkSimPrimer}
===============================

[TOC]

# Introduction #


# Tutorial 1 - a simulation loop #

This tutorial demonstrate how a dynamic simulator can be constructed in c++ 
and how the user can control the simulator.

~~~~{.cpp}
void main(char** 
    // Load dynamic
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/ur_control_test_scene/cup_pg70_table.dwc.xml");

    SerialDeviceController::Ptr devctrl = dwc->findController<SerialDeviceController>("URController");
    Device::Ptr ur = dwc->getWorkcell()->findDevice("UR-6-85-5-A");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );

    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    FKTable table(state);

    // test that the control interface works
    odesim->initPhysics(state);
    Q target(6,0,-0.2,0,0,0,0);
    devctrl->movePTP( target, 100);
    for(int i=0; i<200; i++){
    	std::cout << i << ":";
    	odesim->step(0.01, state);
    	std::cout << ur->getQ(state) << std::endl;
    }
~~~~
