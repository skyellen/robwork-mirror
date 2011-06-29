
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

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwlibs::task;
using namespace rwlibs::proximitystrategies;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::sensor;
using namespace rw::sensor;
using rw::graspplanning::CMDistCCPMeasure3D;
using rw::geometry::GeometryUtil;
using rw::graspplanning::Grasp3D;

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
		_requestSimulationStop(false)
{

	_collisionDetector = ownedPtr(
        new CollisionDetector( dwc->getWorkcell(), ProximityStrategyFactory::makeDefaultCollisionStrategy()));

}

//void GraspTaskSimulator::initialize(){}

void GraspTaskSimulator::stepCB(ThreadSimulator* sim, const rw::kinematics::State& state){
    //std::cout <<sim->getTime() << "    " << std::endl;
    int delay = _stepDelayMs;
    //_simTime = sim->getTime();
    if( delay!= 0 )
        TimerUtil::sleepMs(delay);
    if( _requestSimulationStop ){
        return;
    }

    SimState &sstate = _simStates[sim];
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
        std::cout << "SIMULATION FAILURE0: " << std::endl;
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
            std::cout <<sim->getTime() << " : ";
            std::cout << "TASK FAILURE1: " << mdist << ">" << 0.5 << std::endl;
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
            std::cout << "GRASPING" << std::endl;
            _graspController->setTargetPos(sstate._closeQ);
            sstate._currentState=GRASPING;
            sstate._approachedTime = sim->getTime();
            sstate._restingTime = sstate._approachedTime;

            Transform3D<> t3d  = Kinematics::frameTframe(_tcp, _objects[0]->getBodyFrame(), state);
            sstate._target->getPropertyMap().set<Transform3D<> > ("ObjectTtcpApproach", inverse(t3d) );
        }
    }


    //std::cout << "step callback" << std::endl;
    if(sstate._currentState==GRASPING){
        //std::cout << "grasping" << std::endl;
        if(sim->getTime()> sstate._approachedTime+0.2){
            // test if the grasp is in rest
            bool isResting = DynamicUtil::isResting(_dhand, state, 0.02);
            //std::cout << isResting << "&&" << sim->getTime() << "-" << _restingTime << ">0.08" << std::endl;
            // if it is in rest then lift object
            if( (isResting && ( (sim->getTime()-sstate._restingTime)>0.08)) || sim->getTime()>10 ){
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
                GraspedObject gobj = getObjectContacts(state);
                if( gobj.object == NULL ){
                    _failed++;
                    std::cout << "NEW_GRASP" << std::endl;
                    std::cout << "ObjectMissed" << std::endl;
                    sstate._target->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                    sstate._target->getPropertyMap().set<Q>("QualityBeforeLifting", Q::zero(2));
                    sstate._currentState = NEW_GRASP;
                } else {
                    std::cout << "LIFTING" << std::endl;
                    State nstate = state;
                    Q qualities = calcGraspQuality(state);
                    sstate._target->getPropertyMap().set<Q>("QualityBeforeLifting", qualities);
                    sim->getSimulator()->setTarget(_dhand->getBase(), sstate._wTmbase_retractTarget, nstate);
                    sim->reset(nstate);
                    sstate._currentState = LIFTING;
                }
            }
            if( !isResting ){
            	sstate._restingTime = sim->getTime();
            }

        } else {
        	sstate._restingTime = sim->getTime();
        }
    }

    if(sstate._currentState==LIFTING){
        // test if object has been lifted
        bool isLifted = true;
        Transform3D<> ct3d = Kinematics::worldTframe(_dhand->getBase()->getBodyFrame(), state);
        isLifted &= MetricUtil::dist2( ct3d.P(), sstate._wTretractTarget.P() )<0.005;
        //isLifted &= ct3d.R().equal(_home.R(),0.01);
        //std::cout << MetricUtil::dist2( ct3d.P(), _home.P() ) << "<" << 0.001 << std::endl;
        // if its lifted then verify the object gripper transform
        if (isLifted) {
            GraspedObject gobj = getObjectContacts(state);
            //getTarget()->getPropertyMap().set<Transform3D<> > ("GripperTObject", t3d);
            if( gobj.object == NULL ){
                std::cout << "No contacts!" << std::endl;
                _failed++;
                sstate._target->getPropertyMap().set<int> ("TestStatus", ObjectDropped);
                sstate._target->getPropertyMap().set<int> ("LiftStatus", ObjectDropped);
                sstate._target->getPropertyMap().set<Q>("QualityAfterLifting", Q::zero(2));
            } else {
            	sstate._target->getPropertyMap().set<int> ("LiftStatus", Success);
                Q qualities = calcGraspQuality(state);
                sstate._target->getPropertyMap().set<Q>("QualityAfterLifting", qualities);

                Transform3D<> t3d = Kinematics::frameTframe(_mbase, gobj.object->getBodyFrame(), state);

                // Test the success of lifting the object.
                // We need to look at the objects that are actually touching
                Body* object = gobj.object;
                Body* gripperBody = gobj.bodies[0];
                /*
                Transform3D<> wTp = Kinematics::worldTframe(_mbase->getParent(state), state);

                Transform3D<> objectBefore = gobj.object->getTransformW(_postLiftObjState);
                Transform3D<> objectNow = gobj.object->getTransformW(state);
                Vector3D<> objectMoveVector = objectNow.P() - objectBefore.P();

                Transform3D<> baseBefore = _dhand->getBase()->getTransformW(_postLiftObjState);
                Transform3D<> baseAfter = _dhand->getBase()->getTransformW(state);

                Vector3D<> liftVector = baseAfter.P() - baseBefore.P();
                */

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
                std::cout << "Slippage: " << slippage <<" " << object->getName()<<" " << gripperBody->getName() << std::endl;
                std::cout << "LIFT RESULTS: " << liftResult << std::endl;
                sstate._target->getPropertyMap().set<double> ("LiftResult", liftResult);

                if (liftResult == 0.0) {
                    _failed++;
                    sstate._target->getPropertyMap().set<int> ("TestStatus", ObjectDropped);
                    std::cout << sim->getTime() << " : " << "ObjectDropped" << std::endl;
                } else if (liftResult > 0.50) { // At most 1cm difference with hand lift
                    _success++;
                    sstate._target->getPropertyMap().set<int> ("TestStatus", Success);
                    std::cout << sim->getTime() << " : " << "Success" << std::endl;
                } else {
                    _slipped++;
                    sstate._target->getPropertyMap().set<int> ("TestStatus", ObjectSlipped);
                    std::cout << sim->getTime() << " : " << "ObjectSlipped" << std::endl;
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

        bool colFreeSetup = false;
        do{

            if( !getNextTarget(sstate) ){
                // end we are done with this threadsimulator

            	// make autosave of result
                //saveTasks(true);

                std::cout << "STOP" << std::endl;
                sstate._stopped = true;
                return;
            }


            if( sstate._target->getPropertyMap().get<int>("TestStatus",-1)>=0 ){
                // if test status is set then we allready processed this task.
                _skipped++;
                std::cout << "SKIPPING TARGET - allready processed!\n";
                continue;
            }

            Transform3D<> wTref = Kinematics::frameTframe(_mbase, sstate._taskRefFrame, _homeState);
            Transform3D<> refToffset = sstate._taskOffset;
            Transform3D<> offsetTtarget = sstate._target.get();
            Transform3D<> mbaseTtcp = Kinematics::frameTframe(_mbase, _tcp, _homeState);
            Transform3D<> wTmparent = Kinematics::worldTframe(_mbase->getParent(_homeState), _homeState);

            // and calculate the home lifting position
            sstate._wTtcp_initTarget = wTref * refToffset * offsetTtarget;
            sstate._wTmbase_initTarget     = sstate._wTtcp_initTarget * inverse(mbaseTtcp);
            sstate._wTmbase_approachTarget = sstate._wTtcp_initTarget * sstate._approach * inverse(mbaseTtcp);
            sstate._wTmbase_retractTarget  = sstate._wTtcp_initTarget * sstate._approach * sstate._retract * inverse(mbaseTtcp);

            // we initialize the transform
            _mbase->setTransform( inverse(wTmparent) * sstate._wTmbase_initTarget, nstate);

            //std::cout << "START: " << start << std::endl;
            //std::cout << "HOME : " << _home << std::endl;

            _hand->setQ(sstate._openQ, nstate);
            for(size_t i=0;i<_objects.size();i++){
                Transform3D<> tobj = _objects[i]->getMovableFrame()->getTransform(_homeState);
                _objects[i]->getMovableFrame()->setTransform(tobj, nstate);
            }
            // set max force
            if(_rhand ){
                Q forceLim = sstate._task->getPropertyMap().get<Q>("TauMax",Q());
                if(forceLim.size()>0)
                    _rhand->setForceLimit(forceLim);
            }

            colFreeSetup = !_collisionDetector->inCollision(nstate, NULL, true);

            //std::cout << "Current index: " << (_nextTaskIndex-1) << std::endl;
            if( !colFreeSetup ){
                target->getPropertyMap().set<int>("TestStatus", CollisionInitially);
                target->getPropertyMap().set<Q>("GripperConfiguration", _openQ);

                for(size_t i=0;i<_objects.size();i++){
                    getTarget()->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i),
                                                                      _objects[i]->getTransformW(state));
                }

                //std::cout << "0.0 : InCollision " << std::endl;
                _collision++;
            }
            //std::cout << "1:" << _collision << _nrOfExperiments<< std::endl;

            _nrOfExperiments++;
        } while( !colFreeSetup );

        if( _nrOfExperiments > _lastSaveTaskIndex+40 ){
            saveTasks(true);
            _lastSaveTaskIndex = _nrOfExperiments;
        }
        // reset simulation
        _dhand->getBase()->reset(nstate);
        _tsim->reset(nstate);
        sim->disableBodyControl();
        sim->setTarget(_dhand->getBase(), _approach, nstate);
        _controller->setTargetPos(_openQ);
        _wallTimer.resetAndResume();
        _currentState = APPROACH;
        Transform3D<> t3d  = Kinematics::frameTframe(_tcp, _objects[0]->getBodyFrame(), nstate);
        getTarget()->getPropertyMap().set<Transform3D<> > ("ObjectTtcpTarget", inverse(t3d) );

        _restingTime = 0;
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

GraspTaskSimulator::GraspedObject GraspTaskSimulator::getObjectContacts(const rw::kinematics::State& state)
{
    std::vector<GraspedObject> result;
    for(size_t i=0; i<_objects.size();i++){
        GraspedObject obj;
        obj.object = _objects[i];
        obj.contacts = getObjectContacts(state, _objects[i], _bsensors[i], obj.bodies);
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

const int NR_OF_QUALITY_MEASURES = 3;
rw::math::Q GraspTaskSimulator::calcGraspQuality(const State& state){
    GraspedObject gobj = getObjectContacts(state);
    std::vector<Contact3D> contacts = gobj.contacts;
    RigidBody *object = gobj.object;
    // calculate grasp quality
    rw::math::Q qualities( Q::zero(NR_OF_QUALITY_MEASURES) );
    if(gobj.object==NULL || gobj.contacts.size()==0)
        return qualities;
    Grasp3D g3d( contacts );

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


    std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
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

    std::cout << "w2 "<<std::endl;
    rw::graspplanning::GWSMeasure3D wmeasure2( 10 , false);
    wmeasure2.setObjectCenter(cm);
    wmeasure2.setLambda(1/r);
    wmeasure2.quality(g3d);

    std::cout << "w3 "<<std::endl;
    //

    rw::graspplanning::GWSMeasure3D wmeasure3( 10, true );
    wmeasure3.setObjectCenter(cm);
    wmeasure3.setLambda(1/r);
    wmeasure3.quality(g3d);

    //std::cout << "getvals " << r<< std::endl;
    //std::cout << "Wrench calc done!" << std::endl;
    qualities(0) = wmeasure2.getMinWrench();
    qualities(1) = wmeasure3.getMinWrench();

    std::cout << "CMCPP " << r<< std::endl;
    CMDistCCPMeasure3D CMCPP( cm, r*2);
    qualities(2) = CMCPP.quality( g3d );
    std::cout << "Quality: " << qualities << std::endl;
    return qualities;
}

bool GraspTaskSimulator::getNextTarget(GraspTaskSimulator::SimState){
    // were we iterate over all tasks and their targets

    if(_targets == NULL || _nextTargetIndex>= (int)_targets->size()){
        // get the next task and reinitialize _targets and _currentTaskIndex
        if(_currentTaskIndex+1 >= (int)_taskQueue.size()){
            return NULL; // there is no more tasks
        }
        setTask(_currentTaskIndex+1);
    }
    _currentTargetIndex = _nextTargetIndex;
    Log::logInfo() << "-- target nr: "<< std::setw(5) << _currentTaskIndex
                 << " success:"<< std::setw(5) << _success
                 << " slipped:" << std::setw(5) << _slipped
                 << " failed:" << std::setw(5) << _failed
                 << " collisions:" << std::setw(5) << _collision
                 << " timeouts:" << std::setw(5) << _timeout
                 << " skipped:" << std::setw(5) << _skipped
                 << " simfailures:" << std::setw(5) <<_simfailed << "\n";

    _nextTargetIndex++;
    return (*_targets)[ _currentTargetIndex ];
}

