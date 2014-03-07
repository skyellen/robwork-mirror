
#include <iostream>


#include <rw/rw.hpp>
#include "Robotiq3.hpp"

USE_ROBWORK_NAMESPACE

using namespace robwork;
using namespace rwhw;


int main(int argc, char** argv)
{
    ProgramOptions poptions("SimpleRobotiq3Test1", "0.1");
    poptions.initOptions();
    poptions.parse(argc, argv);

    std::string ip("192.168.100.21");

    PropertyMap map = poptions.getPropertyMap();

    Log::infoLog() << "Initializing Robotiq" << std::endl;

    rwhw::Robotiq3 hand;

    // SETTINGS for communicating with ur 2 on marvin
    std::cout << "Connecting to " << ip << " ... "<< std::endl;
    hand.connect("192.168.100.21", 502);

    // start thread
    //std::cout << "Starting thread.." << std::endl;
    //hand.start();

    std::cout << "Get status" << std::endl;
    hand.getAllStatus();

    //std::cout << "Activating hand.." << std::endl;
    //hand.activate();

    //std::cout << "Gripper activated.." << std::endl;

    //std::cout << "Wait a bit.." << std::endl;
    //TimerUtil::sleepMs(1000);

    std::pair<Q,Q> lim = hand.getLimitPos();
    std::cout << "Move to Open: " << lim.first << std::endl;

    hand.moveCmd(lim.first);
    TimerUtil::sleepMs(5000);
    std::cout << "Move to Close" << std::endl;
    hand.moveCmd(lim.second);
    TimerUtil::sleepMs(5000);

    return 0;
}
