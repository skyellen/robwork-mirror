#include <rw/models/Q.hpp>

#include <rtt/SimulationThread.hpp>
#include <rtt/Event.hpp>
#include <rtt/Handle.hpp>
#include <rtt/TimeService.hpp>
#include <rtt/SimulationActivity.hpp>
#include <rtt/PeriodicActivity.hpp>
#include <rtt/os/startstop.h>

#include <iostream>

using namespace rw::models;

using namespace RTT;

class ControlLoopTest : public RunnableInterface
{

    public:
        ControlLoopTest(){
            _timestamp = TimeService::Instance()->getTicks();
        }

    protected:
        bool initialize() {
            std::cout << "initialize" << std::endl;
            return true;
        }

        void step() {
            Seconds elapsed = TimeService::Instance()->secondsSince( _timestamp );
            std::cout << "step" << " " << elapsed << std::endl;

            //_updateRobotPositionEvent.emit(Q(6));
        }

        void finalize() {
            std::cout << "finalize" << std::endl;
        }

    private:
        TimeService::ticks _timestamp;

    public:
        Event<void(Q)> _updateRobotPositionEvent;
        //...
};


int main( int argc, char** argv)
{
    __os_init(argc, argv);

    ControlLoopTest cltest;

    std::cout << "Press ENTER to proceed to next test" << std::endl;

    std::cout << "Non Preemptible" << std::endl;

    PeriodicActivity own_task( 0, 1.0, &cltest );

    own_task.start();

    fflush(stdin);
    getc(stdin);
    fflush(stdin);

    own_task.stop();

    std::cout << "Simulation" << std::endl;

    //SimulationThread::Instance()->periodSet( 0.001 );
    SimulationThread::Instance()->start();

    SimulationActivity sim( 5.0, &cltest );
    SimulationActivity sim2( 0.5, &cltest );
    sim.start();
    sim2.start();

    //while(true){}
    fflush(stdin);
    getc(stdin);
    sim.stop();
    sim2.stop();

    std::cout << "done" << std::endl;

    __os_exit();

    return 0;
}


