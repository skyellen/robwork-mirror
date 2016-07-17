#include "SimTemplatePlugin.hpp"

#include <RobWorkStudio.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

#include <QTimer>

using namespace rw::common;
using rw::kinematics::State;
using rw::models::WorkCell;
using rws::RobWorkStudioPlugin;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

SimTemplatePlugin::SimTemplatePlugin():
    RobWorkStudioPlugin("SimTemplatePluginUI", QIcon(":/pa_icon.png"))
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

SimTemplatePlugin::~SimTemplatePlugin()
{
}

void SimTemplatePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&SimTemplatePlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SimTemplatePlugin::genericEventListener, this, _1), this);

    Log::setLog( _log );
}

void SimTemplatePlugin::startSimulation() {

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
        ThreadSimulator::StepCallback cb( boost::bind(&SimTemplatePlugin::step, this, _1, _2) );
        _tsim->setStepCallBack( cb );
        _tsim->setRealTimeScale(1.0);
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

    } else {
        _tsim->reset(state);
        log().debug() << "Simulator allready created!";
    }

    _delaySpin->setValue(100);
    _tsim->setRealTimeScale(1.0);
    _wallTimer.resetAndResume();
    _wallTotalTimer.resetAndResume();

    // start poll timer
    _timer->start();
    // start simulator
    _tsim->start();
    log().info() << "Simulation Started\n";
}

void SimTemplatePlugin::open(WorkCell* workcell)
{
    if(workcell==NULL || _dwc==NULL)
        return;
    _startBtn->setEnabled(true);
    _wc = workcell;
}

void SimTemplatePlugin::close() {
    // destroy simulation and stuff
    _wc = NULL;
}

void SimTemplatePlugin::btnPressed() {
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
        int val = _delaySpin->value();
        if(_tsim!=NULL)
            _tsim->setRealTimeScale(val/100.0);
    } else if(obj==_timer){
        // update the RobWorkStudio state
        // we poll the simulator state and updates the visualization state
        getRobWorkStudio()->setState( _tsim->getState() );
    }
}

void SimTemplatePlugin::stateChangedListener(const State& state) {

}

void SimTemplatePlugin::step(ThreadSimulator* sim, const rw::kinematics::State& state){
    State tmpState = state;
    // TODO: in here we are able to perform stuff on the simulation control
    sim->setState(tmpState);
}

void SimTemplatePlugin::genericEventListener(const std::string& event){
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
Q_EXPORT_PLUGIN(SimTemplatePlugin);
#endif
