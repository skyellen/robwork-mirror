
#include <iostream>


#include <rw/rw.hpp>
#include <rwhw/trakstar/Trakstar.hpp>

USE_ROBWORK_NAMESPACE

using namespace robwork;
using namespace rwhw;


int main(int argc, char** argv)
{
    ProgramOptions poptions("Trakstar", "0.1");
    poptions.addStringOption("ini-file", "RobWorkStudio.ini", "RobWorkStudio ini-file");
    poptions.addStringOption("input-file", "", "Project/Workcell/Device input file");
    poptions.setPositionalOption("input-file", -1);
    poptions.initOptions();
    poptions.parse(argc, argv);

    PropertyMap map = poptions.getPropertyMap();

    Trakstar bird;
    Log::infoLog() << "Initializing trakstar sensor" << std::endl;
    bird.initialize();
    Log::infoLog() << "Starting polling of bird sensor" << std::endl;
    bird.startPolling();
    Log::infoLog() << "Logging data" << std::endl;




    int nrSamples = 1*60;
    std::vector<std::string> output(nrSamples);
    for(int i=0;i<nrSamples;i++){
        std::stringstream sstr;
        TimerUtil::sleepMs(8);
        sstr.clear();

        std::vector<Trakstar::PoseData> data = bird.getSensorValues();
        sstr << data.size() << " ";
        BOOST_FOREACH(Trakstar::PoseData& d, data){
            sstr << d.pos[0] << " ";
            sstr << d.pos[1] << " ";
            sstr << d.pos[2] << " ";
            sstr << d.rot(0) << " ";
            sstr << d.rot(1) << " ";
            sstr << d.rot(2) << " ";
            sstr << d.rot(3) << " ";
            sstr << d.quality << " ";
            sstr << d.status << " ";
            sstr << d.valid << " ";
        }
        output[i] = sstr.str();
        std::cout << sstr.str() << std::endl;
    }

    // write all to file
    std::ofstream ofile("trakstar_log.txt");
    BOOST_FOREACH(std::string& str, output){
        ofile << str << "\n";
    }
    ofile.close();
    return 0;
}
