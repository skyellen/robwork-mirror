#include "Task1.hpp"


#include <iostream>
#include <fstream>

using namespace std;
using namespace RTT;
using namespace Template;


Task1::Task1(string name):
    TaskContext(name),

    // add method
    _setCounter("setCounter", &Task1::setCounter, this),

    // name the commands
    _command1("command1", &Task1::command1, &Task1::command_completion, this),

    // name the dataport
    _counterPort("counter", 0)

{
    // add method and command
    log(Info) << this->getName() << ": Add method and command" << endlog();


    methods()->addMethod(&_setCounter, "set the counter", "Parameter 1", "Integer");
 
    commands()->addCommand(&_command1, "command 1", "parameter1", "integer");
    
    ports()->addPort(&_counterPort);

}


Task1::~Task1()
{
}


bool Task1::startup()    
{
    log(Info) << this->getName() << ": Startup" << endlog();
    return true;
}


void Task1::update()
{
    log(Info) << this->getName() << ": Update" << endlog();
}


void Task1::shutdown()
{
    log(Info) << this->getName() << ": Shutdown" << endlog();
}


void Task1::setCounter(int counter)
{
    _counterPort.Set(counter);

    log(Info) << this->getName() << ": set counter to" <<counter<< endlog();
}



bool Task1::command1(int b)
{
    log(Info) << this->getName() << ": command1(" << b <<")"<< endlog();
    return true;
}


bool Task1::command_completion() const
{
    return true;
}


