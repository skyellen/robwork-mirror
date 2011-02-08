#include "SamplePlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

#include <rwlibs/simulation/SimulatedController.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;

using namespace rws;

using namespace rwlibs::simulation;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    _timer = new QTimer( NULL );
    _timer->setInterval( 200 );
    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );
}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
    std::cout << "SamplePlugin::initialize" << std::endl;
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SamplePlugin::genericEventListener, this, _1), this);


}
void SamplePlugin::startSimulation() {
    // START
    State state = getRobWorkStudio()->getState();
    _hand =_wc->findDevice<JointDevice>("PG70");
    _dhand = _dwc->findDevice("PG70");
    if(_hand==NULL)
        return;

    Frame *f = _hand->getBase();
    _mbase = dynamic_cast<MovableFrame*>(f);
    if(_mbase==NULL)
        return;

    _object = _dwc->findBody<RigidBody>("Object");
    if(_object==NULL)
        return;

    Frame *end = _wc->findFrame("TCPPG70");
    if(end==NULL)
        return;

    _startQ = _hand->getQ(state);
    _startQ[0] = 0.035;
    _closeQ = _startQ;
    _openQ = _startQ;
    _closeQ[0] = 0;

    _bTe = Kinematics::frameTframe(_mbase, end, state);
    _wTe_n = _tasks->getPropertyMap().get<Transform3D<> >("Nominal");
    _wTe_home = _tasks->getPropertyMap().get<Transform3D<> >("Home");
    _nextTaskIndex = 0;
    _objHome = _object->getMovableFrame()->getTransform(state);
    _targets = &_tasks->getTargets();
    _stopped = false;
    SimulatedController* scontroller = _dwc->findController("GraspController").get();
    if(scontroller==NULL) RW_THROW("No controller!");
    _controller = dynamic_cast<rwlibs::control::JointController*>( scontroller->getController() );
    _progressBar->setRange( 0 , _targets->size());
    _progressBar->setValue( 0 );
    _tsim->setPeriodMs( _speedSpinBox->value() );
    _currentState = NEW_GRASP;
    std::cout << "Starttimer" << std::endl;
    _timer->start();
    std::cout << "Start sim" << std::endl;
    _tsim->start();
    std::cout << "sim started" << std::endl;

    log().info() << "Simulation Started\n";
}
void SamplePlugin::open(WorkCell* workcell)
{
    std::vector<std::string> ids = PhysicsEngineFactory::getEngineIDs();
    std::cout << "Available PhysicsEngines: \n";
    BOOST_FOREACH(std::string id, ids){ std::cout << id << "\n"; }
    if(workcell==NULL || _dwc==NULL)
        return;
    _wc = workcell;
    _bodyController = new rwsim::control::BodyController("BodyController");
    _dwc->addController( _bodyController );
    // create simulator and stuff here
    makeSimulator();
    // load tasks from file
    loadTasks();
    //start simulation

    if(getRobWorkStudio()->getPropertyMap().has("Auto")){
        startSimulation();
    }
}

void SamplePlugin::close() {
}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_btn0){
        startSimulation();
    } else if(obj==_btn1){
        log().info() << "Button 1\n";

        saveTasks();

    } else if(obj==_timer){
        // update the RobWorkStudio state
        State state = _tsim->getState();
        getRobWorkStudio()->setState(state);

        if(_stopped){
            _tsim->stop();
            _timer->stop();
            saveTasks();
        }

        // update progress
        _progressBar->setValue( _nextTaskIndex );

    }
}

void SamplePlugin::stateChangedListener(const State& state) {

}
/**

-   Move the gripper in the start transform
-   If collision in start pose
    o   Set TestStatus = CollisionInitially
    o   Continue;
-   Simulate the gripper closing
-   If distance between jaws=0
    o   Set TestStatus = ObjectMissed
    o   Continue
-   Simulate moving to home pose
-   If object dropped
    o   Set TestStatus = ObjectDropped
    o   Continue
-   Set TestStatus = Success
-   Store Gripper Configuration



 */

void SamplePlugin::step(const rw::kinematics::State& state){
    std::cout <<"\r" <<_sim->getTime() << "   ";
    if( _stopped ){
        return;
    }
    if(_sim->getTime()>5.0){
        (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
        (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", TimeOut);
        _currentState = NEW_GRASP;
    }

    Transform3D<> cT3d = Kinematics::worldTframe(_object->getBodyFrame(), state);

    if(_currentState!=NEW_GRASP ){
        if( MetricUtil::dist2(_objHome.P(),cT3d.P())>0.4 ){
            std::cout << "TASK FAILURE" << std::endl;
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            _currentState = NEW_GRASP;
        } else if( _objHome.P()[2] > cT3d.P()[2]+0.02 ){
            std::cout << "TASK FAILURE" << std::endl;
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            _currentState = NEW_GRASP;
        }
    }

    //std::cout << "step callback" << std::endl;
    if(_currentState==GRASPING){
        //std::cout << "grasping" << std::endl;
        _graspedQ = _hand->getQ(state);
        if(_sim->getTime()>0.2){
            // test if the grasp is in rest
            bool isResting = DynamicUtil::isResting(_dwc, state);
            // if it is in rest then lift object
            if( (isResting && ( (_sim->getTime()-_restingTime)>0.08)) || _sim->getTime()>3 ){
                // remember to check the transform of object relative to gripper
                _restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
                _graspTime = _sim->getTime();


                // now instruct the RigidBodyController to move the object to the home configuration
                State nstate = state;
                //_mbase->setTransform(_home, nstate);

                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
                if(_graspedQ[0]<0.001){
                    (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                    _currentState = NEW_GRASP;
                } else {
                    _bodyController->setTarget(_dhand->getBase(), _home, nstate);
                    _tsim->setState(nstate);
                    _currentState = LIFTING;
                }
            }
            if( !isResting ){
                _restingTime = _sim->getTime();
            }

            if(_graspedQ[0]<0.001){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                _currentState = NEW_GRASP;
            }

        } else {
            _restingTime = _sim->getTime();
        }
    }

    if(_currentState==LIFTING){
        // test if object has been lifted
        bool isLifted = true;
        Transform3D<> ct3d = Kinematics::worldTframe(_dhand->getBase()->getBodyFrame(), state);
        isLifted &= MetricUtil::dist2( ct3d.P(), _home.P() )<0.001;
        //isLifted &= ct3d.R().equal(_home.R(),0.01);

        // if its lifted then verify the object gripper transform
        if(isLifted){
            Transform3D<> t3d = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
            _graspedQ = _hand->getQ(state);
            if(_graspedQ[0]<0.001){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectDropped);
            } else if( /*_restObjTransform.R().equal(t3d.R(), 0.01 ) &&*/
                    (MetricUtil::dist2(_restObjTransform.P(),t3d.P())<0.006 ) ) {
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", Success);
            } else {
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectSlipped);
            }
            _currentState = NEW_GRASP;
        }
        _graspedQ = _hand->getQ(state);

        if(_graspedQ[0]<0.001){
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectDropped);
            _currentState = NEW_GRASP;
        }
    }

    if(_currentState==NEW_GRASP){
        State nstate = getRobWorkStudio()->getState();
        // pop new task from queue
        // if all tasks

        bool colFreeSetup = false;
        do{
            if(_nextTaskIndex>=(int)_targets->size()){
                // end we are done
                _stopped = true;
                std::cout << "STOP" << std::endl;
                // save the result
                return;
            }

            Transform3D<> start = _wTe_n * (*_targets)[_nextTaskIndex]->get() * inverse(_bTe);
            _mbase->setTransform(start, nstate);
            // and calculate the home lifting position
            _home = _wTe_home * (*_targets)[_nextTaskIndex]->get() * inverse(_bTe);
            _hand->setQ(_openQ, nstate);
            _object->getMovableFrame()->setTransform(_objHome, nstate);

            colFreeSetup = !getRobWorkStudio()->getCollisionDetector()->inCollision(nstate, NULL, true);
            _nextTaskIndex++;

            if( !colFreeSetup ){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", CollisionInitially);
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _openQ[0]);
                std::cout << "in collision: " << _nextTaskIndex << std::endl;
            }

            //std::cout << "current index: " << _nextTaskIndex << std::endl;
        } while( !colFreeSetup );

        // reset simulation
        _bodyController->reset(nstate);
        _dhand->getBase()->reset(nstate);
        _tsim->setState(nstate);
        _controller->setTargetPos(_closeQ);
        _currentState = GRASPING;
        _restingTime = 0;
    }
}
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwlibs/opengl/Drawable.hpp>
void SamplePlugin::makeSimulator(){
    // we have a DWC create the simulator
    State state = getRobWorkStudio()->getState();
    log().debug() << "Making physics engine";
    ODESimulator::Ptr _engine = ownedPtr( new ODESimulator(_dwc));
    log().debug() << "Making simulator";
    _sim = ownedPtr( new DynamicSimulator(_dwc, _engine ));
    log().debug() << "Initializing simulator";
    _sim->init(state);
    log().debug() << "Creating Thread simulator";

    _tsim = ownedPtr( new ThreadSimulator(_sim, state) );

    ThreadSimulator::StepCallback cb( boost::bind(&SamplePlugin::step, this, _1) );
    _tsim->setStepCallBack( cb );

    _tsim->setPeriodMs(0);
    _tsim->setTimeStep(0.001);

    rwsim::drawable::SimulatorDebugRender::Ptr debugRender = _sim->createDebugRender();
    if( debugRender == NULL ){
        Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
        return;
    }
    debugRender->setDrawMask( 1 );
    rwlibs::opengl::Drawable *debugDrawable = new rwlibs::opengl::Drawable( debugRender, "DebugRender" );
    getRobWorkStudio()->getWorkCellScene()->addDrawable(debugDrawable, _dwc->getWorkcell()->getWorldFrame());

}


void SamplePlugin::loadTasks(){
    std::string filename = getRobWorkStudio()->getPropertyMap().get<std::string>("TaskTestFile", "test.task.xml");
    std::cout << "Loading tasks: ";
    std::cout << "\t-Filename: " << filename;

    try {
        XMLTaskLoader loader;
        loader.load(filename);
        _tasks = loader.getCartesianTask();
    } catch (const Exception& exp) {
        std::cout << "UNABLE TO LOAD  TASKS" << std::endl;

        QMessageBox::information(this, "Task Execution Widget", "Unable to load tasks");
        return;
    }
    std::cout << "LOAD TASKS DONE: " << _tasks->getTargets().size() << std::endl;

}
#include <fstream>
#include <iostream>

void SamplePlugin::exportMathematica() {
           std::string filename = getRobWorkStudio()->getPropertyMap().get<std::string>("TaskTestOutFile", "test.task.txt");
           std::stringstream sstr;
           sstr << filename << ".mathematica.result.txt";

           log().info() << "Saving tasks: ";
           log().info() << "\t-Filename: " << sstr.str();

           std::ofstream outfile(sstr.str().c_str());

           std::vector<CartesianTarget::Ptr> targets = _tasks->getTargets();
           BOOST_FOREACH(CartesianTarget::Ptr target, targets) {
                      const Vector3D<>& pos = target->get().P();
                      const RPY<> rpy(target->get().R());
                      int status = target->getPropertyMap().get<int>("TestStatus", UnInitialized);
                      double distance = target->getPropertyMap().get<double>("GripperConfiguration", -1);
                      outfile<<"{"<<pos(0)<<","<<pos(1)<<","<<pos(2)<<","<<rpy(0)<<","<<rpy(1)<<","<<rpy(2)<<","<<status<<","<<distance<<"}"<<std::endl;
           }
           outfile.close();


}


void SamplePlugin::saveTasks(){
    std::string filename = getRobWorkStudio()->getPropertyMap().get<std::string>("TaskTestOutFile", "test.task.xml");
    std::stringstream sstr;
    sstr << filename << ".result.xml";

    std::cout << "Saving tasks: \n";
    std::cout  << "\t-Filename: " << sstr.str() << "\n";

    try {
        XMLTaskSaver saver;
        //saver.save(_tasks, "task_test_result.xml");
        saver.save(_tasks, sstr.str());

    } catch (const Exception& exp) {
        //QMessageBox::information(this, "Task Execution Widget", "Unable to load tasks");
    }
    exportMathematica();
}


void SamplePlugin::genericEventListener(const std::string& event){
    if( event=="DynamicWorkcellLoadet" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");

        DynamicWorkCell::Ptr dwc =
            getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell",NULL);

        if( dwc==NULL){
            std::cout << "Could not load dynamic workcell from propertymap!!" << std::endl;
            return;
        }
        //std::cout << "dwc" << dwc->getWorkCell()->getName() << std::endl;
        _dwc = dwc;
    }
}

Q_EXPORT_PLUGIN(SamplePlugin);
