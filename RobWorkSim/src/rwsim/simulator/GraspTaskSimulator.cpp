
#include "GraspTaskSimulator.hpp"

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/graspplanning/Grasp3D.hpp>
#include <rwlibs/task.hpp>
#include <fstream>
#include <iostream>
#include <stack>
#include <boost/lexical_cast.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rw/graspplanning/CMDistCCPMeasure3D.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwlibs::task;
using namespace rwlibs::proximitystrategies;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::control;
using namespace rwsim::sensor;
using namespace rw::sensor;
using rw::graspplanning::CMDistCCPMeasure3D;
using rw::geometry::GeometryUtil;
using rw::graspplanning::Grasp3D;
using rwlibs::simulation::SimulatedController;

const int NR_OF_QUALITY_MEASURES = 5;

namespace {
    double getMaxObjectDistance(std::vector<RigidBody*> objects, const State& s1, const State& s2){
        double max = 0;
        BOOST_FOREACH(RigidBody *object, objects){
            Transform3D<> t1 = object->getTransformW(s1);
            Transform3D<> t2 = object->getTransformW(s2);
            if(MetricUtil::dist2(t1.P(),t2.P())>max)
                max = MetricUtil::dist2(t1.P(),t2.P());
        }
        return max;
    }
}


GraspTaskSimulator::GraspTaskSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dwc):
        _dwc(dwc),
		_requestSimulationStop(false),
		_stepDelayMs(0),
		_autoSaveInterval(40),
		_maxObjectGripperDistanceThreshold(50),
		_stat(SimulationFailure),
		_initialized(false),
		_nrOfThreads(1),
		_currentTargetIndex(0),
		_alwaysResting(false)
{
}

GraspTaskSimulator::~GraspTaskSimulator(){

}

void GraspTaskSimulator::init(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const rw::kinematics::State& initState){
    _dwc = dwc;
    _collisionDetector = ownedPtr(
        new CollisionDetector( dwc->getWorkcell(), ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // initialize simulators
    _simStates.clear();
    _simulators.clear();

    for(int i=0;i<_nrOfThreads;i++){
        //Log::debugLog() << "Making physics engine";
        ODESimulator::Ptr engine = ownedPtr( new ODESimulator(_dwc));
        //Log::debugLog() << "Making simulator";
        DynamicSimulator::Ptr sim = ownedPtr( new DynamicSimulator(_dwc, engine ));
        //Log::debugLog() << "Initializing simulator";
        try {
            State istate = initState;
            sim->init(istate);
        } catch(...){
            RW_THROW("could not initialize simulator!\n");
        }
        //Log::debugLog() << "Creating Thread simulator";

        ThreadSimulator::Ptr tsim = ownedPtr( new ThreadSimulator(sim, initState) );
        ThreadSimulator::StepCallback cb( boost::bind(&GraspTaskSimulator::stepCB, this, _1, _2) );

        tsim->setStepCallBack( cb );
        tsim->setPeriodMs(-1);
        tsim->setTimeStep(0.01);

        _simulators.push_back(tsim);
        _homeState = initState;
    }

    _initialized = true;
}


void GraspTaskSimulator::load(const std::string& filename){
    Log::infoLog() << "Loading tasks: ";
    Log::infoLog() << "\t-Filename: " << filename;
    GraspTask::Ptr task;
    try {
        GraspTask::load( filename );
    } catch (const Exception& exp) {
        RW_WARN("Unable to load tasks from file! " << filename);
        return;
    }
    load(task);
}

void GraspTaskSimulator::load(GraspTask::Ptr graspTasks){
    while(!_taskQueue.empty())
        _taskQueue.pop();
    _gtask = graspTasks;
    _roottask = graspTasks->getRootTask();
    _taskQueue.push(_roottask);
    _currentTask = NULL;

    int nrOfTargets = 0;
    std::stack<rwlibs::task::CartesianTask::Ptr> tmpStack;
    tmpStack.push(_roottask);
    while(!tmpStack.empty()){
        rwlibs::task::CartesianTask::Ptr tmpTask = tmpStack.top();
        tmpStack.pop();

        nrOfTargets += tmpTask->getTargets().size();

        //BOOST_FOREACH(CartesianTarget::Ptr target, tmpTask->getTargets()){
        //    _alltargets.push_back( std::make_pair(tmpTask,target));
        //}

        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, tmpTask->getTasks()){
            tmpStack.push(subtask);
        }
    }
    _currentTargetIndex = 0;
    _totalNrOfExperiments = nrOfTargets;

    _objects = _dwc->findBodies<RigidBody>();
    std::string handName = _roottask->getPropertyMap().get<std::string>("Gripper");
    _dhand = _dwc->findDevice(handName);
    if(_dhand==NULL)
        RW_THROW("No such gripper in workcell: " << handName);
    _rhand = dynamic_cast<RigidDevice*>(_dhand);
    _hand = _dhand->getKinematicModel();
    _gripperDim = _hand->getDOF();

    // hbase is the
    _hbase = dynamic_cast<KinematicBody*>( _dhand->getBase() );
    if(_hbase==NULL)
        RW_THROW("The gripper base must be a KinematicBody: " << handName);
    _mbase = _hbase->getMovableFrame();

    std::string controllerName = _roottask->getPropertyMap().get<std::string>("ControllerName", "GraspController");
    SimulatedController* scontroller = _dwc->findController(controllerName).get();
    if(scontroller==NULL)
        RW_THROW("No controller exist with the name: " << controllerName);
    _graspController = dynamic_cast<rwlibs::control::JointController*>( scontroller->getController() );
    if(_graspController==NULL)
        RW_THROW("Only JointControllers are valid graspcontrollers!");

    std::string tcpName = _roottask->getPropertyMap().get<std::string>("TCP");
    _tcp = _dwc->getWorkcell()->findFrame(tcpName);

    Log::infoLog() << "LOAD TASKS DONE, nr of tasks: " << nrOfTargets;
}

//----- simulation control and query function api
void GraspTaskSimulator::startSimulation(const rw::kinematics::State& initState){
    _nrOfExperiments = 0;
    if(!_initialized)
        init(_dwc, initState);

    if(_totalNrOfExperiments==0){
        _requestSimulationStop = true;
        return;
        //RW_THROW("there are no tasks to simulate!");
    }

    _requestSimulationStop = false;

    _failed = 0;
    _success = 0;
    _slipped = 0;
    _collision = 0;
    _timeout = 0;
    _simfailed = 0;
    _skipped = 0;
    _nrOfExperiments = 0;
    _lastSaveTaskIndex = 0;

    // remove all sensors if there are any
    // first remove any SimTaskObjectSensor from the simulator
    for(size_t i=0;i<_simulators.size();i++){
        if(_simStates.find(_simulators[i])!= _simStates.end() ){
            BOOST_FOREACH(BodyContactSensor::Ptr sensor, _simStates[_simulators[i]]._bsensors){
                _simulators[i]->getSimulator()->removeSensor(sensor);
            }
        }
    }

    _simStates.clear();

    while(!_taskQueue.empty())
        _taskQueue.pop();
    _taskQueue.push(_roottask);
    _currentTask = NULL;
    _currentTargetIndex = 0;
    _homeState = initState;


    // FOR NOW WE ONLY USE ONE THREAD
    for(size_t i=0;i<_simulators.size();i++){
        DynamicSimulator::Ptr sim = _simulators[i]->getSimulator();
        SimState sstate;
        sstate._state = _homeState;

        _hbase->getMovableFrame()->setTransform(Transform3D<>(Vector3D<>(100,100,100)), sstate._state);

        for(size_t j=0;j<_objects.size();j++){
            sstate._bsensors.push_back( ownedPtr(new BodyContactSensor("SimTaskObjectSensor", _objects[j]->getBodyFrame())) );
            sim->addSensor( sstate._bsensors.back() , sstate._state);
        }

        _simulators[i]->setPeriodMs(-1);
        _simulators[i]->setTimeStep(0.01);
        _simStates[_simulators[i]] = sstate;
    }
    for(size_t i=0;i<_simulators.size();i++){
        _simulators[i]->start();
    }
}

void GraspTaskSimulator::pauseSimulation(){
    _requestSimulationStop = true;
    BOOST_FOREACH(ThreadSimulator::Ptr tsim, _simulators){
        tsim->stop();
    }
}

void GraspTaskSimulator::resumeSimulation(){
    _requestSimulationStop = false;
    BOOST_FOREACH(ThreadSimulator::Ptr tsim, _simulators){
        tsim->start();
    }
}

bool GraspTaskSimulator::isRunning(){

    BOOST_FOREACH(ThreadSimulator::Ptr tsim, _simulators){
        if(tsim->isRunning())
            return true;
    }
    return false;
}

bool GraspTaskSimulator::isFinished(){
    //std::cout << "_totalNrOfExperiments==_nrOfExperiments" << _totalNrOfExperiments<<"=="<<_nrOfExperiments << "\n";
    return (_totalNrOfExperiments==_nrOfExperiments) && !isRunning();
}

size_t GraspTaskSimulator::getNrTargets(){
    return _totalNrOfExperiments;
}

ThreadSimulator::Ptr GraspTaskSimulator::getSimulator(){
    if(_simulators.size()>0)
        return _simulators[0];
    return NULL;
}

std::vector<ThreadSimulator::Ptr> GraspTaskSimulator::getSimulators(){
    return _simulators;
}

int GraspTaskSimulator::getNrTargetsDone(){
    return _nrOfExperiments;
}

//void GraspTaskSimulator::initialize(){}

void GraspTaskSimulator::stepCB(ThreadSimulator* sim, const rw::kinematics::State& state){
    SimState &sstate = _simStates[sim];
    //std::cout <<sim->getTime() << "   state: " << sstate._currentState << std::endl;
    int delay = _stepDelayMs;
    //_simTime = sim->getTime();
    if( delay!= 0 )
        TimerUtil::sleepMs(delay);
    if( _requestSimulationStop ){
        return;
    }


    sstate._state = state;

    Q currentQ = _hand->getQ(state);

    if(sstate._wallTimer.getTime()>60){ //seconds
        _timeout++;
        sstate._target->getPropertyMap().set<Q>("GripperConfiguration", currentQ);
        sstate._target->getPropertyMap().set<int>("TestStatus", TimeOut);
        sstate._currentState = NEW_GRASP;
    }

    if(sim->getTime()>20.0 && sstate._currentState != NEW_GRASP){
        _timeout++;
        sstate._target->getPropertyMap().set<Q>("GripperConfiguration", currentQ);
        sstate._target->getPropertyMap().set<int>("TestStatus", TimeOut);
        sstate._currentState = NEW_GRASP;
    }

    //Transform3D<> cT3d = Kinematics::worldTframe(_object->getBodyFrame(), state);
    if( sim->isInError() ) {
        // the simulator is in error, reinitialize or fix the error
        _simfailed++;
        //std::cout << "SIMULATION FAILURE0: " << std::endl;
        sstate._target->getPropertyMap().set<Q>("GripperConfiguration", currentQ);
        sstate._target->getPropertyMap().set<int>("TestStatus", SimulationFailure);
        //_restObjState = state;
        for(size_t i=0; i<_objects.size(); i++){
            Transform3D<> restTransform = Kinematics::frameTframe(_mbase, _objects[i]->getBodyFrame(), state);
            sstate._target->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i), restTransform);
        }
        sstate._currentState = NEW_GRASP;
    }

    if(sstate._currentState!=NEW_GRASP ){
        if( getMaxObjectDistance( _objects, _homeState, state) > _maxObjectGripperDistanceThreshold ){
            double mdist=getMaxObjectDistance( _objects, _homeState, state);
            _simfailed++;
            //std::cout <<sim->getTime() << " : ";
            //std::cout << "TASK FAILURE1: " << mdist << ">" << 0.5 << std::endl;
            sstate._target->getPropertyMap().set<Q>("GripperConfiguration", currentQ);
            sstate._target->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            for(size_t i=0; i<_objects.size(); i++){
                Transform3D<> restTransform = Kinematics::frameTframe(_mbase, _objects[i]->getBodyFrame(), state);
                sstate._target->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i), restTransform);
            }
            sstate._currentState = NEW_GRASP;
        }
    }

    if(sstate._currentState ==APPROACH){
        Transform3D<> ct3d = Kinematics::worldTframe(_mbase, state);
        bool isLifted = MetricUtil::dist2( ct3d.P(), sstate._wTmbase_approachTarget.P() )<0.002;
        //std::cout << MetricUtil::dist2( ct3d.P(), _approach.P() ) << " < " << 0.002 << std::endl;
        //if(sim->getTime()>1.2){

        //std::cout << "APPROACH: " << std::endl;
        if(isLifted){
            //std::cout << "GRASPING" << std::endl;
            _graspController->setTargetPos(sstate._closeQ);
            sstate._currentState=GRASPING;
            sstate._approachedTime = sim->getTime();
            sstate._restingTime = sstate._approachedTime;
            sstate._restCount = 0;
            Transform3D<> t3d  = Kinematics::frameTframe(_tcp, _objects[0]->getBodyFrame(), state);
            sstate._target->getPropertyMap().set<Transform3D<> > ("ObjectTtcpApproach", inverse(t3d) );
        }
    }


    //std::cout << "step callback" << std::endl;
    if(sstate._currentState==GRASPING){
        //std::cout << "grasping" << std::endl;
        if(sim->getTime()> sstate._approachedTime+0.2){
            // test if the grasp is in rest
            Log::infoLog() << _alwaysResting;
            if(DynamicUtil::isResting(_dhand, state, 0.0001, 0.1) /*|| _alwaysResting*/)
                sstate._restCount++;

            bool isResting = sstate._restCount > 15;
            //std::cout << isResting << "&& (" << sim->getTime() << "-" << sstate._restingTime << ">0.4) || " << sim->getTime() << ">" << 10 << std::endl;
            // if it is in rest then lift object
            if( (isResting && ( (sim->getTime()-sstate._restingTime)>0.4)) || sim->getTime()>10 ){
                // remember to check the transform of object relative to gripper
                //_restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
                sstate._graspTime = sim->getTime();
                sstate._postLiftObjState = state;
                //_objectBeginLift = _object->getBodyFrame()->getTransform(state);
                // now instruct the RigidBodyController to move the object to the home configuration

                //_mbase->setTransform(_home, nstate);
                sstate._target->getPropertyMap().set<Q>("GripperConfiguration", currentQ);
                for(size_t i=0;i<_objects.size();i++){
                	sstate._target->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i),
                                                                      _objects[i]->getTransformW(state));
                }
                GraspedObject gobj = getObjectContacts(state, sstate);
                if( gobj.object == NULL ){
                    _failed++;
                    //std::cout << "NEW_GRASP" << std::endl;
                    //std::cout << "ObjectMissed" << std::endl;
                    sstate._target->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                    sstate._target->getPropertyMap().set<Q>("QualityBeforeLifting", Q::zero(NR_OF_QUALITY_MEASURES));
                    sstate._currentState = NEW_GRASP;
                } else {
                    std::cout << "LIFTING" << std::endl;
                    State nstate = state;
                    Q qualities = calcGraspQuality(state, sstate);
                    sstate._target->getPropertyMap().set<Q>("QualityBeforeLifting", qualities);
                    sim->getSimulator()->setTarget(_dhand->getBase(), sstate._wTmbase_retractTarget, nstate);
                    sim->reset(nstate);
                    sstate._currentState = LIFTING;
                    sstate._restCount = 0;
                }
            }
            if( sstate._restCount == 3 ){
            	sstate._restingTime = sim->getTime();
            }

        } else {
        	sstate._restingTime = sim->getTime();
        }
    }

    if(sstate._currentState==LIFTING){
        // test if object has been lifted
        //std::cout << _dhand->getVelocity(state) << std::endl;
        bool isLifted = true;
        Transform3D<> ct3d = Kinematics::worldTframe(_dhand->getBase()->getBodyFrame(), state);
        isLifted &= MetricUtil::dist2( ct3d.P(), sstate._wTmbase_retractTarget.P() )<0.001;
        if(isLifted)
            sstate._restCount++;
        //isLifted &= ct3d.R().equal(_home.R(),0.01);
        //std::cout << MetricUtil::dist2( ct3d.P(), sstate._wTmbase_retractTarget.P() ) << "<" << 0.001 << std::endl;
        // if its lifted then verify the object gripper transform
        if (isLifted && sstate._restCount>50) {
            GraspedObject gobj = getObjectContacts(state, sstate);
            //getTarget()->getPropertyMap().set<Transform3D<> > ("GripperTObject", t3d);
            if( gobj.object == NULL ){
                //std::cout << "No contacts!" << std::endl;
                _failed++;
                sstate._target->getPropertyMap().set<int> ("TestStatus", ObjectDropped);
                sstate._target->getPropertyMap().set<int> ("LiftStatus", ObjectDropped);
                sstate._target->getPropertyMap().set<Q>("QualityAfterLifting", Q::zero(2));
            } else {
            	sstate._target->getPropertyMap().set<int> ("LiftStatus", Success);
                Q qualities = calcGraspQuality(state, sstate);
                sstate._target->getPropertyMap().set<Q>("QualityAfterLifting", qualities);

                Transform3D<> t3d = Kinematics::frameTframe(_mbase, gobj.object->getBodyFrame(), state);

                // Test the success of lifting the object.
                // We need to look at the objects that are actually touching
                Body* object = gobj.object;
                Body* gripperBody = gobj.bodies[0];

                Transform3D<> tcpTo_before = Kinematics::frameTframe(_tcp, object->getBodyFrame(), sstate._postLiftObjState);
                Transform3D<> tcpTo_after  = Kinematics::frameTframe(_tcp, object->getBodyFrame(), state);
                sstate._target->getPropertyMap().set<Transform3D<> > ("ObjectTtcpGrasp", inverse(tcpTo_before) );
                sstate._target->getPropertyMap().set<Transform3D<> > ("ObjectTtcpLift", inverse(tcpTo_after) );


                Transform3D<> oTg_before = Kinematics::frameTframe(object->getBodyFrame(), gripperBody->getBodyFrame(), sstate._postLiftObjState);
                Transform3D<> oTg_after  = Kinematics::frameTframe(object->getBodyFrame(), gripperBody->getBodyFrame(), state);
                Vector3D<> slipVector = oTg_after.P() - oTg_before.P();
                // allow op to 2 cm slip else its a fault

                double slippage = slipVector.norm2();

                double liftResult;
                if(slippage <= 0.02)
                    liftResult = (0.02 - slippage)*50;
                else
                    liftResult = 0.0;
                //std::cout << "Slippage: " << slippage <<" " << object->getName()<<" " << gripperBody->getName() << std::endl;
                //std::cout << "LIFT RESULTS: " << liftResult << std::endl;
                sstate._target->getPropertyMap().set<double> ("LiftResult", liftResult);
                sstate._restCount = 0;
                if (liftResult == 0.0) {
                    _failed++;
                    sstate._target->getPropertyMap().set<int> ("TestStatus", ObjectDropped);
                    //std::cout << sim->getTime() << " : " << "ObjectDropped" << std::endl;
                } else if (liftResult > 0.50) { // At most 1cm difference with hand lift
                    _success++;
                    sstate._target->getPropertyMap().set<int> ("TestStatus", Success);
                    //std::cout << sim->getTime() << " : " << "Success" << std::endl;
                } else {
                    _slipped++;
                    sstate._target->getPropertyMap().set<int> ("TestStatus", ObjectSlipped);
                    //std::cout << sim->getTime() << " : " << "ObjectSlipped" << std::endl;
                }

            }
            sstate._target->getPropertyMap().set<Q>("GripperConfigurationPost", currentQ);
            for(size_t i=0;i<_objects.size();i++){
            	sstate._target->getPropertyMap().set<Transform3D<> >("GripperTObjectLift"+boost::lexical_cast<std::string>(i),
                                                                  _objects[i]->getTransformW(state));
            }
            sstate._currentState = NEW_GRASP;
        }
    }

    if(sstate._currentState==NEW_GRASP){
        State nstate = _homeState;
        // pop new task from queue
        // if all tasks
        //RW_WARN("1");
        bool colFreeSetup = false;
        do{
            //RW_WARN("1");
            if( !getNextTarget(sstate) ){
                std::cout << "STOPP" << std::endl;
                // end we are done with this threadsimulator
                sstate._stopped = true;
                sim->postStop();
                // stop the thread
                return;
            }
            //RW_WARN("1");

            if( sstate._target->getPropertyMap().get<int>("TestStatus",-1)>=0 ){
                // if test status is set then we allready processed this task.
                _skipped++;
                _nrOfExperiments++;
                colFreeSetup=false;
                continue;
            }

            Transform3D<> wTref = Kinematics::worldTframe(sstate._taskRefFrame, _homeState);
            Transform3D<> refToffset = sstate._taskOffset;
            Transform3D<> offsetTtarget = sstate._target->get();
            Transform3D<> mbaseTtcp = Kinematics::frameTframe(_mbase, _tcp, _homeState);
            Transform3D<> wTmparent = Kinematics::worldTframe(_mbase->getParent(_homeState), _homeState);
            //std::cout << "mbase parent: "<< _mbase->getParent(_homeState)->getName() << std::endl;
            //RW_WARN("1");
            // and calculate the home lifting position
            sstate._wTtcp_initTarget = wTref * refToffset * offsetTtarget;
            sstate._wTmbase_initTarget     = sstate._wTtcp_initTarget * inverse(mbaseTtcp);
            sstate._wTmbase_approachTarget = sstate._wTtcp_initTarget * sstate._approach * inverse(mbaseTtcp);
            //sstate._wTmbase_retractTarget  = sstate._wTtcp_initTarget * sstate._approach * sstate._retract * inverse(mbaseTtcp);
            Transform3D<> wTretract =  wTref * refToffset * sstate._retract;
            sstate._wTmbase_retractTarget  = wTretract * sstate._wTmbase_approachTarget;
            //RW_WARN("1");
            // we initialize the transform
            //std::cout << "inverse(wTmparent)" << inverse(wTmparent) << std::endl;

            //std::cout << "inverse(wTmparent)" << sstate._wTmbase_initTarget << std::endl;
            //std::cout << "inverse(wTmparent)" << sstate._wTmbase_initTarget << std::endl;
            _mbase->setTransform( inverse(wTmparent) * sstate._wTmbase_initTarget, nstate);
            //RW_WARN("1");

            _hand->setQ(sstate._openQ, nstate);
            for(size_t i=0;i<_objects.size();i++){
                Transform3D<> tobj = _objects[i]->getMovableFrame()->getTransform(_homeState);
                _objects[i]->getMovableFrame()->setTransform(tobj, nstate);
            }
            //RW_WARN("1");
            // set max force
            if(_rhand ){
                Q forceLim = sstate._task->getPropertyMap().get<Q>("TauMax",Q());
                if(forceLim.size()>0)
                    _rhand->setForceLimit(forceLim);
            }
            //RW_WARN("1");
            colFreeSetup = !_collisionDetector->inCollision(nstate, NULL, true);
            //RW_WARN("1");
            if( !colFreeSetup ){
                sstate._target->getPropertyMap().set<int>("TestStatus", CollisionInitially);
                sstate._target->getPropertyMap().set<Q>("GripperConfiguration", sstate._openQ);

                for(size_t i=0;i<_objects.size();i++){
                    sstate._target->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i),
                                                                         _objects[i]->getTransformW(state));
                }

                //std::cout << "0.0 : InCollision " << std::endl;
                _collision++;
            }
            //RW_WARN("1");
            //std::cout << "1:" << _collision << _nrOfExperiments<< std::endl;

            _nrOfExperiments++;
        } while( !colFreeSetup );

        if( _nrOfExperiments > _lastSaveTaskIndex+_autoSaveInterval ){
            //saveTasks(true);
            _lastSaveTaskIndex = _nrOfExperiments;
        }
        // reset simulation
        _dhand->getBase()->reset(nstate);
        sim->reset(nstate);
        sim->setState(nstate);

        sim->getSimulator()->disableBodyControl();
        sim->getSimulator()->setTarget(_dhand->getBase(), sstate._wTmbase_approachTarget, nstate);
        _graspController->setTargetPos(sstate._openQ);
        sstate._wallTimer.resetAndResume();
        sstate._currentState = APPROACH;
        Transform3D<> t3d  = Kinematics::frameTframe(_tcp, _objects[0]->getBodyFrame(), nstate);
        sstate._target->getPropertyMap().set<Transform3D<> > ("ObjectTtcpTarget", inverse(t3d) );

        sstate._restingTime = 0;
    }
}


namespace {

	void exportMathematica(std::ostream& outfile, CartesianTask::Ptr tasks, int gripperDim) {
		   outfile << "// Description: {target.pos(3), target.rpy(3), TestStatus(1), GripperConfiguration("<<gripperDim<<"), "
				   "GripperTObject.pos, GripperTObject.rpy, ObjectTtcpBefore.pos, ObjectTtcpBefore.rpy, ObjectTtcpAfter.pos, ObjectTtcpAfter.rpy}\n";
		   outfile << "// TestStatus enum { UnInitialized=0, Success=1, CollisionInitially=2, ObjectMissed=3, ObjectDropped=4, ObjectSlipped=5, TimeOut=6, SimulationFailure=7}\n";
		   std::stack<CartesianTask::Ptr> taskStack;
		   taskStack.push(tasks);
		   while(!taskStack.empty()){
			   CartesianTask::Ptr task = taskStack.top();
			   taskStack.pop();

			   std::vector<CartesianTarget::Ptr> targets = task->getTargets();
			   outfile<<"{" << task->getId() << "}\n";
			   BOOST_FOREACH(CartesianTarget::Ptr target, targets) {
				  const Vector3D<>& pos = target->get().P();
				  const RPY<> rpy(target->get().R());
				  int status = target->getPropertyMap().get<int>("TestStatus", GraspTaskSimulator::UnInitialized);
				  outfile<<"{"<<pos(0)<<","<<pos(1)<<","<<pos(2)<<","<<rpy(0)<<","<<rpy(1)<<","<<rpy(2)<<","<<status<<",";

				  Q distance = target->getPropertyMap().get<Q>("GripperConfiguration", Q::zero(gripperDim));
				  for(size_t i=0;i<distance.size();i++)
					  outfile << distance[i] << ",";

				  Transform3D<> t3d = target->getPropertyMap().get<Transform3D<> >("GripperTObject", Transform3D<>::identity());
				  RPY<> rpyObj(t3d.R());
				  outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
					  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << ",";

				  t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget", Transform3D<>::identity() );
				  rpyObj = RPY<>(t3d.R());
				  outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
					  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2)<< ",";

				  t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpApproach", Transform3D<>::identity() );
				  rpyObj = RPY<>(t3d.R());
				  outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
					  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2)<< ",";

				  t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpGrasp", Transform3D<>::identity() );
				  rpyObj = RPY<>(t3d.R());
				  outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
					  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << ",";

				  t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpLift", Transform3D<>::identity() );
				  rpyObj = RPY<>(t3d.R());
				  outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
					  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << "}"<< "\n";
			   }

		   }

	}

}
/*
void GraspTaskSimulator::save(const std::string& filename, CartesianTask::Ptr tasks, ExportFormat format){
	std::ofstream outfile(filename.c_str());
	save(outfile, tasks, format);
	outfile.close();
}

void GraspTaskSimulator::save(std::ostream& ostr, CartesianTask::Ptr tasks, ExportFormat format){
	int gripperDim = 0;
	if( format==TaskFormat ){
	    try {
	        XMLTaskSaver saver;
	        saver.save(tasks, ostr );
	    } catch (const Exception& exp) {
	    	RW_THROW("Unable to save task: " << exp.what());
	    }
	} else if( format==CommaSeperatedFormat ){
		exportMathematica(ostr, tasks, gripperDim);
	} else {
		RW_THROW("Unkown Export Format!");
	}
}
*/


std::vector<rw::sensor::Contact3D> GraspTaskSimulator::getObjectContacts(const rw::kinematics::State& state,
		RigidBody *object,
		BodyContactSensor::Ptr sensor,
		std::vector<Body*>& gripperbodies )
{
    const std::vector<rw::sensor::Contact3D>& contacts = sensor->getContacts();
    const std::vector<Body*>& bodies = sensor->getBodies();

    RW_ASSERT(bodies.size() == contacts.size() );
    //std::cout << "nr contacts: " << contacts.size() << " body: " << object->getName() << std::endl;
    std::vector<rw::sensor::Contact3D> contactres;
    std::map<std::string, Frame*> frameTree = Kinematics::buildFrameMap( *_hand->getBase(),  state);
    frameTree[_hand->getBase()->getName()] = _hand->getBase();
    for(size_t i=0; i<bodies.size(); i++){
        if( bodies[i]!=NULL ){
            // test that the body frame is part of the gripper
            //std::cout << "Body: " << bodies[i]->getBodyFrame()->getName() << std::endl;
            if( frameTree.find(bodies[i]->getBodyFrame()->getName() ) != frameTree.end() ){
                if(contacts[i].normalForce>0.0001){
                    contactres.push_back(contacts[i]);
                    contactres.back().mu = _dwc->getMaterialData().getFrictionData(object->getMaterialID(),bodies[i]->getMaterialID()).parameters[0].second(0);
                    // allso save the body of the gripper that is in contact
                    if(std::find(gripperbodies.begin(), gripperbodies.end(), bodies[i]) == gripperbodies.end() )
                        gripperbodies.push_back(bodies[i]);
                }
            }
        } else {
            //std::cout << "Body: NULL" << std::endl;
        }
    }
    //std::cout << "Get CONTACTS " << contacts.size() << " --> " << contactres.size() << std::endl;
    return contactres;
}

GraspTaskSimulator::GraspedObject GraspTaskSimulator::getObjectContacts(const rw::kinematics::State& state, SimState &sstate)
{
    std::vector<GraspedObject> result;
    for(size_t i=0; i<_objects.size();i++){
        GraspedObject obj;
        obj.object = _objects[i];
        obj.contacts = getObjectContacts(state, _objects[i], sstate._bsensors[i], obj.bodies);
        if(obj.contacts.size()>0)
            result.push_back(obj);
    }
    if(result.size()==0)
        return GraspedObject();
    int bestIdx = 0;
    for(size_t i=1;i<result.size();i++){
        if( result[i].contacts.size() > result[bestIdx].contacts.size() )
            bestIdx = i;
    }
    return result[bestIdx];
}


rw::math::Q GraspTaskSimulator::calcGraspQuality(const State& state, SimState &sstate){
    GraspedObject gobj = getObjectContacts(state, sstate);
    std::vector<Contact3D> contacts = gobj.contacts;
    RigidBody *object = gobj.object;
    // calculate grasp quality
    rw::math::Q qualities( Q::zero(NR_OF_QUALITY_MEASURES) );
    if(gobj.object==NULL || gobj.contacts.size()==0)
        return qualities;
    Grasp3D g3d( contacts );

    /*
    if(g3d.contacts.size()<4){
        std::vector<Contact3D > cons = g3d.contacts;
        BOOST_FOREACH(Contact3D& c, cons){
            // add a small random value to normal and position
            c.n += Vector3D<>(Math::ran(-0.1,0.1), Math::ran(-0.1,0.1),Math::ran(-0.1,0.1));
            c.n = normalize(c.n);
            c.p += Vector3D<>(Math::ran(-0.002,0.002), Math::ran(-0.002,0.002),Math::ran(-0.002,0.002));
            g3d.contacts.push_back(c);
        }
    }
    */

    //std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
    /*
    BOOST_FOREACH(Contact3D con, g3d.contacts){
        std::cout << "--- contact";
        std::cout << "\n-- nf: " << con.normalForce;
        std::cout << "\n-- mu: " << con.mu;
        std::cout << "\n-- n : " << con.n;
        std::cout << "\n-- p: " << con.p << std::endl;
    }
    */

    Vector3D<> cm = object->getInfo().masscenter;
    double r = GeometryUtil::calcMaxDist( object->getGeometry(), cm);
    //std::cout << "cm    : " << cm << std::endl;
    //std::cout << "Radius: " << r<< std::endl;

    rw::graspplanning::GWSMeasure3D wmeasure2( 10 , false);
    wmeasure2.setObjectCenter(cm);
    wmeasure2.setLambda(1/r);
    wmeasure2.quality(g3d);

    rw::graspplanning::GWSMeasure3D wmeasure3( 10, true );
    wmeasure3.setObjectCenter(cm);
    wmeasure3.setLambda(1/r);
    wmeasure3.quality(g3d);

    //std::cout << "getvals " << r<< std::endl;
    //std::cout << "Wrench calc done!" << std::endl;
    qualities(0) = wmeasure2.getMinWrench();
    qualities(1) = wmeasure2.getAvgWrench();
    qualities(2) = wmeasure3.getMinWrench();
    qualities(3) = wmeasure3.getAvgWrench();

    //std::cout << "CMCPP " << r<< std::endl;
    CMDistCCPMeasure3D CMCPP( cm, r*2);
    qualities(4) = CMCPP.quality( g3d );
    //std::cout << "Quality: " << qualities << std::endl;
    return qualities;
}

bool GraspTaskSimulator::getNextTarget(GraspTaskSimulator::SimState& sstate){
    // were we iterate over all tasks and their targets
    //RW_WARN("1");

    if(_currentTask==NULL || (_currentTargetIndex>=_currentTask->getTargets().size()-1)){
        //RW_WARN("1");

        // if the current target is the last target then get the next task
        do{
            if(_taskQueue.empty() ){
                //RW_WARN("1");
                _currentTask = NULL;
                return false;
            }

            _currentTask = _taskQueue.top();
            _taskQueue.pop();

            // push all task children on the queue
            for(int i=_currentTask->getTasks().size()-1; i>=0; i--){
                _taskQueue.push(_currentTask->getTasks()[i]);
            }

        } while(_currentTask->getTargets().size()==0);
        _currentTargetIndex=0;

    } else {
        //RW_WARN("1");
        _currentTargetIndex++;
    }
    //RW_WARN("1");

    if(sstate._task!= _currentTask){
        //RW_WARN("1");
        sstate._task = _currentTask;
        // TODO: compute task temp variables

        std::string refframename = _currentTask->getPropertyMap().get<std::string>("refframe","WORLD");
        sstate._taskRefFrame = _dwc->getWorkcell()->findFrame<Frame>(refframename);
        RW_ASSERT(sstate._taskRefFrame);
        sstate._taskOffset = _currentTask->getPropertyMap().get<Transform3D<> >("Offset",Transform3D<>::identity() );
        sstate._approach = _currentTask->getPropertyMap().get<Transform3D<> >("Approach",Transform3D<>::identity() );
        sstate._retract = _currentTask->getPropertyMap().get<Transform3D<> >("Retract",Transform3D<>::identity() );

        sstate._openQ = _currentTask->getPropertyMap().get<Q>("OpenQ");
        sstate._closeQ = _currentTask->getPropertyMap().get<Q>("CloseQ");
    }
    //RW_WARN("1");
    // and target specific temp variables
    sstate._target = _currentTask->getTargets()[_currentTargetIndex];
    //RW_WARN("1");
    //sstate._wTtcp_initTarget,
    //                        _wTmbase_initTarget, // approach to this config from _initTarget
    //                        _wTmbase_approachTarget, // approach to this config from _initTarget
    //                        _wTmbase_retractTarget; // retract to this config from _approachTarget



    Log::infoLog() << "-- target nr: "<< std::setw(5) << _currentTargetIndex
                 << " success:"<< std::setw(5) << _success
                 << " slipped:" << std::setw(5) << _slipped
                 << " failed:" << std::setw(5) << _failed
                 << " collisions:" << std::setw(5) << _collision
                 << " timeouts:" << std::setw(5) << _timeout
                 << " skipped:" << std::setw(5) << _skipped
                 << " simfailures:" << std::setw(5) <<_simfailed << "\n";

    return true;
}

