/**
 * This example shows how to use StateMachines in Orocos. The main method
 * constructs a TaskContext for controlling the StateMachine and two 
 * TaskContexts (Task1 and Task2), which are controlled from the StateMachine.
 * 
 *
 *
 */
#include <rtt/RTT.hpp>
#include <rtt/PeriodicActivity.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/main.h>
#include <ocl/TaskBrowser.hpp>
#include "Task1.hpp"
#include "Task2.hpp"


using namespace Orocos;
using namespace Template;


int ORO_main(int arc, char* argv[])
{
    // Set log level more display Info messages as well
    if (log().getLogLevel() < Logger::Info ) 
        log().setLogLevel( Logger::Info );
    

    // task 1
    Task1 task1("task1");
    PeriodicActivity task1_activity(5, 1.0, task1.engine());

    // task 2
    Task2 task2("task2");
    PeriodicActivity task2_activity(5, 2.0, task2.engine());

    
    // state machine task
    TaskContext statemachine("StateMachine");
    PeriodicActivity statemachine_activity(RTT::OS::HighestPriority,0.1, statemachine.engine());
    

    // connect data flow
    connectPorts(&task1, &task2);

    // connect execution flow
    connectPeers(&statemachine, &task1);
    connectPeers(&statemachine, &task2);

    // load scripts
    statemachine.scripting()->loadStateMachines("StateMachine.osd");

    // start tasks
    statemachine_activity.start();

    
    // active state machine
    statemachine.engine()->states()->getStateMachine("Default")->activate();

    // start state machine (automatic mode) :
    statemachine.engine()->states()->getStateMachine("Default")->start();

    while (statemachine.isRunning());

    return 0;
}
