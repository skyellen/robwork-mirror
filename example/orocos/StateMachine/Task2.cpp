#include "Task2.hpp"


using namespace std;
using namespace RTT;
using namespace Template;


Task2::Task2(string name):
    TaskContext(name),
    // name the methods
    _getCounter("getCounter", &Task2::getCounter, this),
    
    // name the port
    _counterPort("counter")
{
 
    // register methods
    log(Info) << this->getName() << ": Add method" << endlog();


    methods()->addMethod(&_getCounter, "get counter");

    ports()->addPort(&_counterPort);
   
}


Task2::~Task2()
{
}


bool Task2::startup()    
{
    log(Info) << this->getName() << ": Started" << endlog();
    return true;
}


void Task2::update()
{
    log(Info) << this->getName() << ": Update" << endlog();
}


void Task2::shutdown()
{
    log(Info) << this->getName() << ": Shutdown" << endlog();
}



int Task2::getCounter()
{
    log(Info) << this->getName() << ": getCounter() returns "<<_counterPort.Get()<<endlog();
    return _counterPort.Get();
}

