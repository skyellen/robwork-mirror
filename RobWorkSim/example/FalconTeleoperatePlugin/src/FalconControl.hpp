#ifndef FALCON_CONTROL_HPP
#define FALCON_CONTROL_HPP

#include "FalconInterface.hpp"
#include "IO.hpp"

#include <RobWorkStudio.hpp>

#include "ui_FalconControl.h"

#include <rw/rw.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/control/SerialDeviceController.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactStrategyPQP.hpp>
#include <rwsim/contacts/Contact.hpp>

#include <QPushButton>

using namespace std;

using namespace rw::kinematics;
using namespace rwsim::dynamics;
using namespace rwsim::control;
using namespace rwsim::contacts;
using namespace rwlibs::control;
using namespace rwlibs::simulation;


USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
/**
 * @brief A plugin
 */


class FalconControl: public rws::RobWorkStudioPlugin, private Ui::FalconControl
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

    /**
     * @brief constructor
     */
    FalconControl();

    //! destructor
    virtual ~FalconControl();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);
    void keyEventListener(int key, Qt::KeyboardModifiers modifier);

    void makeSimulator();
    void step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);
    void startSimulation();
    
    
private slots:
    void btnPressed();
    void stateChangedListener(const rw::kinematics::State& state);
    void saveFile();

private:
    rw::models::WorkCell* _wc;
    rw::kinematics::Frame* _tcpFrame;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rwsim::simulator::ThreadSimulator::Ptr _tsim;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    rwsim::simulator::ODESimulator::Ptr _engine;
	rw::kinematics::State _state;
    QTimer *_timer;
    SerialDeviceController::Ptr controller;
    PDController::Ptr controllerPG;
    Device::Ptr dev;
	vector<Body::Ptr> allBodies;
    
	FalconInterface falcon;
	
	int _rotateYaw,_rotatePitch;

	int recordCount;	//record output count. Initially 0.
	Recorder rec;		//to record the conf. poses and contact points.
	double totalTime;	//passed time since beginning

};

#endif
