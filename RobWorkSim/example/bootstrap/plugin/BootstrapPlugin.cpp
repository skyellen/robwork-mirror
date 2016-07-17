#include "BootstrapPlugin.hpp"

#include <bstrap/core/Brain.hpp>
#include <bstrap/abstractions/VisionSensor.hpp>

#include <RobWorkStudio.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwsim/control/PoseController.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

#include <QTimer>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace rws;

using namespace rwsim::control;
using namespace rwlibs::control;
using namespace rwlibs::simulation;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

BootstrapPlugin::BootstrapPlugin():
    RobWorkStudioPlugin("BootstrapPlugin", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_showTaskSpinBox    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

    // this is the timer used to make events where we poll the simulator for information
    _timer = new QTimer( NULL );
    _timer->setInterval( 100 );

    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );

    _startBtn->setEnabled(false);
    _stopBtn->setEnabled(false);
}

BootstrapPlugin::~BootstrapPlugin()
{
}

void BootstrapPlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&BootstrapPlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&BootstrapPlugin::genericEventListener, this, _1), this);

    Log::setLog( _log );
}

void BootstrapPlugin::startSimulation() {

    // create simulator and stuff here
    log().info() << "Making simulation!";
    State state = getRobWorkStudio()->getState();
    if(_sim==NULL){
        log().debug() << "Making physics engine";
        ODESimulator::Ptr engine = ownedPtr( new ODESimulator(_dwc));
        log().debug() << "Making simulator";
        _sim = ownedPtr( new DynamicSimulator(_dwc, engine ));
        log().debug() << "Initializing simulator";
        try {
            _sim->init(state);
        } catch(...){
            log().error() << "could not initialize simulator!\n";
        }
        log().debug() << "Creating Thread simulator";
        _tsim = ownedPtr( new ThreadSimulator(_sim, state) );
        ThreadSimulator::StepCallback cb( boost::bind(&BootstrapPlugin::step, this, _1, _2) );
        _tsim->setStepCallBack( cb );
        //_tsim->setPeriodMs(100);
        _tsim->setTimeStep(0.01);

        // if we want some debug information visualized then we add a render
        rwsim::drawable::SimulatorDebugRender::Ptr debugRender = _sim->createDebugRender();
        if( debugRender == NULL ){
            Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
            return;
        }

        debugRender->setDrawMask( 7 );
        rwlibs::opengl::Drawable *debugDrawable = new rwlibs::opengl::Drawable( debugRender, "DebugRender" );

        getRobWorkStudio()->getWorkCellScene()->addDrawable(debugDrawable, _dwc->getWorkcell()->getWorldFrame());

        // find the head/vision frame
        Frame *vision = _wc->findFrame("VisionFrame");
        PoseController::Ptr leftCtrl = _dwc->findController<PoseController>("LeftArmPoseController");
        PoseController::Ptr rightCtrl = _dwc->findController<PoseController>("RightArmPoseController");

        //_brain = new Brain("TheBrain", _tsim, _sim, _dwc, this);
        _brain = new Brain("TheBrain", this);
        // add some Abstractions, schemas and motor programs to the brain
        //_brain->add( ownedPtr( new VisionSensor( vision, _tsim, _sim, _dwc )) );
        _brain->add( ownedPtr( new VisionSensor( vision, _dwc )) );

        // add some grasp schemas


        // lets start the brain straight away
        _brain->start();

        // We push a button and execute a schema
        //Schema *schema = new Schema("myschema", _tsim, _sim, _dwc, this);
        //schema->start();

    } else {
        _tsim->reset(state);
        log().debug() << "Simulator allready created!";
    }

    _delaySpin->setValue(100);
    //_tsim->setPeriodMs(100);
    //_wallTimer.resetAndResume();
    //_wallTotalTimer.resetAndResume();

    // start poll timer
    _timer->start();
    // start simulator
    _tsim->start();
    log().info() << "Simulation Started\n";
}

void BootstrapPlugin::open(WorkCell* workcell)
{
    if(workcell==NULL || _dwc==NULL)
        return;
    _startBtn->setEnabled(true);
    _wc = workcell;
}

void BootstrapPlugin::close() {
    // destroy simulation and stuff
    _wc = NULL;
}

void BootstrapPlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_startBtn){
        startSimulation();
        _startBtn->setEnabled(false);
        _stopBtn->setEnabled(true);
    } else if(obj==_stopBtn){
        _startBtn->setEnabled(true);
        _stopBtn->setEnabled(false);
        _tsim->stop();
        _timer->stop();
    } else if(obj==_delaySpin){
    	RW_THROW("Delay is not currently implemented.");
        //int val = _delaySpin->value();
        //if(_tsim!=NULL)
            //_tsim->setPeriodMs(val);
    } else if(obj==_timer){
        // update the RobWorkStudio state
        // we poll the simulator state and updates the visualization state
        getRobWorkStudio()->setState( _tsim->getState() );
    }
}

void BootstrapPlugin::stateChangedListener(const State& state) {

}

void BootstrapPlugin::step(ThreadSimulator* sim, const rw::kinematics::State& state){
    /*
    State tmpState = state;
    // in here we are able to perform stuff on the simulation control
    KinematicBody *body = _dwc->findBody<KinematicBody>("PG70.Base");
    PDController::Ptr jc = _dwc->findController<PDController>("GraspController");
    PoseController::Ptr jp = _dwc->findController<PoseController>("URPoseController");
    if (body != NULL) {
        //body->setForce( Vector3D<>(0,1,0), tmpState );

        _sim->setTarget(body, Transform3D<>(Vector3D<>(0, 0, 2)), tmpState);

    }

    if(jc!=NULL){
        std::cout << ((int)sim->getTime()) % 2 << std::endl;
        Q q = jc->getModel().getQ(state);
        if( ((int)sim->getTime()) % 2 ){
            jc->setTargetPos( jc->getModel().getBounds().first );
        } else {
            jc->setTargetPos( jc->getModel().getBounds().second );
        }
    }

    if(jp!=NULL){
        std::cout << ((int)sim->getTime()) % 2 << std::endl;
        if( ((int)sim->getTime()) % 2 ){
            jp->setTarget( Transform3D<>( Vector3D<>(0,0.5,0.7) ) );
        } else {
            jp->setTarget( Transform3D<>( Vector3D<>(0,-0.5,0.7) ) );
        }
    }

    sim->setState(tmpState);
    */
}

void BootstrapPlugin::genericEventListener(const std::string& event){
    if( event=="DynamicWorkcellLoadet" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");
        DynamicWorkCell::Ptr dwc =
            getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell",NULL);

        if( dwc==NULL){
            log().error() << "Could not load dynamic workcell from propertymap!!" << std::endl;
            return;
        }
        _dwc = dwc;
    }
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(BootstrapPlugin);
#endif
