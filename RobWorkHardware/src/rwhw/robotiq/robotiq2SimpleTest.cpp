#include <iostream>

#include <rw/common/TimerUtil.hpp>
#include <boost/program_options.hpp>

#include "Robotiq2.hpp"
namespace po = boost::program_options;

USE_ROBWORK_NAMESPACE

using namespace robwork;
using namespace rwhw;

int main(int argc, char** argv)
{
    po::options_description desc("Demo to show the robotiq3 hand moving!");
    desc.add_options()
        ("debug,d", "Enable debug messages!")
        ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (...) {
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if (vm.count("debug")) {
        Log::log().setLevel(Log::Debug);
    }

    if (vm.count("help")) {
          std::cout << desc << std::endl;
          return -1;
    }

    // SETTINGS for communicating with robotiq2 on marvin
    std::string ip("192.168.100.22");

    Log::infoLog() << "Initializing Robotiq" << std::endl;

    rwhw::Robotiq2 hand;

    std::cout << "Connecting to " << ip << " ... " << std::endl;
    hand.connect(ip);

    std::cout << "Get status" << std::endl;
    hand.getAllStatusCMD();
    std::cout << "Get hand limit" << std::endl;
    std::pair<Q, Q> lim = hand.getLimitPos();
    std::cout << "Joint values " << hand.getQ() << std::endl;

    std::cout << "Move to Open: " << lim.first << std::endl;

    hand.moveCmd(lim.first);
    hand.getAllStatusCMD();
    std::cout << "Joint values while moving " << hand.getQ() << std::endl;

    TimerUtil::sleepMs(5000);
    hand.getAllStatusCMD();
    std::cout << "Joint values. Should be open " << hand.getQ() << std::endl;

    std::cout << "Move to Close " << lim.second << std::endl;
    hand.moveCmd(lim.second);
    TimerUtil::sleepMs(500);
    std::cout << "Stop mid Close" << std::endl;
    hand.stopCmd();
    TimerUtil::sleepMs(5000);
    std::cout << "Resume Close" << std::endl;
    hand.moveCmd();
    TimerUtil::sleepMs(1000);

    std::cout << "Move to Open: " << lim.first << std::endl;
    hand.moveCmd(lim.first);
    TimerUtil::sleepMs(500);
    std::cout << "Stop and disconnect" << std::endl;
    hand.stopCmd();
    hand.disconnect();

    return 0;
}
