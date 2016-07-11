#include "RWSimulator.hpp"

#include "ContactModelFactory.hpp"
#include "ContactGraph.hpp"
#include "ConstantForceManipulator.hpp"
#include "RWDebugRender.hpp"

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/KinematicDevice.hpp>

#include <rw/common/TimerUtil.hpp>

#include <boost/foreach.hpp>

#include "EulerIntegrator.hpp"

#include "CNodePool.hpp"

#include "SequintialImpulseSolver.hpp"


// #define RWSIMULATOR_DEBUG

using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rwsim;
using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rwsim;

using namespace rwlibs::simulation;

//#define RW_DEBUG( str ) std::cout << str  << std::endl;

#define TIMING( str, func ) \
    { long start = (long)rw::common::TimerUtil::currentTimeMs(); \
    func; \
     long end = (long)rw::common::TimerUtil::currentTimeMs(); \
    std::cout << str <<":" << (end-start) <<"ms"<<std::endl;  }

//#define TIMING( str, func ) {func;}

namespace {

    void saveState(const RWBodyList& bodies, double dt, State &state){
        // ------------------------ STATE UPDATE STEP ---------------

        // TODO: Constraint controlled bodies should be updated here
        //_contactGraph->saveState();
        // now update the states of the contact free bodies
        //std::cout << "Updating body states" << std::endl;
        // simple euler update
        //std::cout << "Nr of bodies: " << _bodies.size() << std::endl;
        BOOST_FOREACH(RWBody *body, bodies){
            body->saveState(dt, state);
        }
    }

}

RWSimulator::RWSimulator():
    _dwc(NULL),
    _time(0),
    _frameToBody(NULL,100)
{

}

RWSimulator::RWSimulator(dynamics::DynamicWorkCell::Ptr dwc):
    _dwc(dwc),
    _time(0),
    _frameToBody(NULL,100)
{

}


void RWSimulator::load(dynamics::DynamicWorkCell::Ptr dwc){
    _dwc = dwc;
}

bool RWSimulator::setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector) {
	return false;
}

void RWSimulator::initPhysics(State& state){
    // create constraint nodes and constraint edges to a CNodePool
    RW_DEBUG("- Allocating CNodePool");
    _pool = new CNodePool();

    // create ContactGraph from the dwc
    RW_DEBUG("- Creating constraint nodes for free bodies");
    std::vector<ConstraintNode*> node;
    BOOST_FOREACH(Body::Ptr b, _dwc->getBodies() ){
        Body *body = b.get();
        if( dynamic_cast<RigidBody*>(body) ){
            ConstraintNode *node = _pool->createCNode( ConstraintNode::Rigid );
            if(node==NULL)
                continue;
            //node->setFrame( &(body->getBodyFrame()) );
            RWBody *rwbody = _bodyPool.createBody( RWBody::Rigid );
            rwbody->setBody(body);

            node->setBody( rwbody );
            _bodies.push_back( rwbody );
            // TODO: choose an integrator for the body using file properties
            _integrators.push_back( new EulerIntegrator( rwbody ) );
        } else if( dynamic_cast<FixedBody*>(body) ) {
            ConstraintNode *node = _pool->createCNode( ConstraintNode::Fixed );
            if(node==NULL)
                continue;

            RWBody *rwbody = _bodyPool.createBody( RWBody::Fixed );
            rwbody->setBody(body);

            node->setFrame( body->getBodyFrame() );
            node->setBody( rwbody );
        } else if( dynamic_cast<KinematicBody*>(body) ) {
            ConstraintNode *node = _pool->createCNode( ConstraintNode::Scripted );
            if(node==NULL)
                continue;
            RWBody *rwbody = _bodyPool.createBody( RWBody::Kinematic );

            rwbody->setBody(body);

            node->setFrame( body->getBodyFrame() );
            node->setBody( rwbody );
        }
    }

    RW_DEBUG("- Creating constraint nodes and edges from devices!");
    // construct nodes from
    BOOST_FOREACH(DynamicDevice::Ptr dev, _dwc->getDynamicDevices()){
        if( dynamic_cast<RigidDevice*>(dev.get()) ){
            std::cout << "NO SUPPORT FOR RIGID DEVICES!" << std::endl;
        } else if( dynamic_cast<KinematicDevice*>(dev.get()) ){
            std::cout << "NO SUPPORT FOR KINEMATIC DEVICES!" << std::endl;
        }
    }

    // Create gravity manipulator
    _gravityManipulator = new ConstantForceManipulator(_dwc->getGravity(), _bodies);
    _manipulators.push_back(_gravityManipulator);

    RW_DEBUG("Creating contact model factory");
    _factory = new ContactModelFactory( _dwc.get(), _pool );
    RW_DEBUG("Creating contact graph");
    _cgraph = new ContactGraph( _pool, *_factory );
    RW_DEBUG("Creating contact solver");
    _solver = new SequintialImpulseSolver();

    _controllers = _dwc->getControllers();
}

void RWSimulator::resetScene(State& state){

}

void RWSimulator::exitPhysics(){
    // destruct every thing...

}

drawable::SimulatorDebugRender::Ptr RWSimulator::createDebugRender() {
	return ownedPtr( new RWDebugRender(*_dwc) );
}

void RWSimulator::step(double dt, State& state){
    double totalEnergyBefore=0,totalEnergyAfter=0;
    // calculate energy before
    BOOST_FOREACH(RWBody *body, _bodies){
        totalEnergyBefore += body->calcEnergy(_dwc->getGravity());
    }

    Simulator::UpdateInfo info;
    info.time = _time;
    info.dt = dt;
    info.rollback = false;
    RW_DEBUG("* Update all controllers!");
    BOOST_FOREACH(SimulatedController::Ptr controller, _controllers ){
        controller->update(info, state);
    }

    // TODO: do device updates

    RW_DEBUG("* Internal step");
    double timeStep = internalStep(dt, state);

    // update sensors

    // And energy after
    RW_DEBUG("* Energy calculation: ");
    BOOST_FOREACH(RWBody *body, _bodies){
        totalEnergyAfter += body->calcEnergy(_dwc->getGravity());
    }

    RW_DEBUG("* TOTAL ENERGY: ");
    RW_DEBUG("** before = "<< totalEnergyBefore );
    RW_DEBUG("** after  = "<< totalEnergyAfter );
    _time += timeStep;
}

double RWSimulator::internalStep(double dt, rw::kinematics::State& state){
    double timeStep = dt;

    // ------------------ FORCE RESET --------------------------------
    // first reset all force contributions to zero
    RW_DEBUG("* Reset forces");
    BOOST_FOREACH(RWBody *body, _bodies){
        RW_ASSERT(body);
        body->reset();
        body->calcAuxVarialbles(state);
    }

    RW_DEBUG("* Add external forces ");
    BOOST_FOREACH(BodyController* manipulator, _manipulators) {
        manipulator->addForces( state , _time );
    }

    RW_DEBUG("* Calculate contact points ");
    TIMING( "** time: ", _cgraph->updateContacts( state ) );

    RW_DEBUG("* Calculate contact groups ");
    std::vector<CEdgeGroup> groups = _cgraph->computeGroups();
    RW_DEBUG("** nr of groups: " << groups.size() );
    //addContactForces(state, groups, _dt);

    // save current state so as to enable rollback
    _cgraph->saveState();
    saveState( _bodies, timeStep, state );
    State savedState = state;
    bool penetrating = false;


    RW_DEBUG("* PERFORMING COLLISION RESOLUTION");
    do {
        // now update velocities
        RW_DEBUG("** update velocities ");
        BOOST_FOREACH(BodyIntegrator *integrator, _integrators){
            integrator->updateVelocity(timeStep, state);
        }

        RW_DEBUG("** --------------->UPDATE COLLISION FORCES< ");
        RW_DEBUG("** Add collision impulses ");

        // Impulse solving
        SolverInfo sinfo(timeStep);
        _solver->solve(groups, sinfo, state); // solve constraints

        // *****************************************************************
        // update position state
        RW_DEBUG("** update position ");
        BOOST_FOREACH(BodyIntegrator *integrator, _integrators){
            integrator->updatePosition(timeStep, state);
        }

        // *****************************************************************
        //RW_DEBUG("** Performing Broad phase update of contact graph");
        RW_DEBUG("** Broad phase");
        TIMING("*** time: ", _cgraph->broadPhase( state, false) );

        RW_DEBUG("** Narrow phase");
        TIMING("*** time: ", penetrating = _cgraph->narrowPhase( state, false) );

        if(penetrating){
            //std::cout << "* rollback to last stable state " << std::endl;
            state = savedState;
            rollBack( state );

            timeStep = timeStep/2;
            RW_DEBUG("** PENETRATING TRUE, ROLLBACK: " << _time);
            RW_DEBUG("** Reduce dt: " << timeStep);
        }
    } while(penetrating);

    BOOST_FOREACH(RWBody *body, _bodies){
        std::cout << "VELOCITY OF BODY: " << body->getBodyFrame()->getName() << std::endl;
        std::cout << "LinVel: " <<body->getLinVel() << std::endl;
        std::cout << "AngVel: " <<body->getAngVel() << std::endl;
    }

    // update time
    return timeStep;
}

void RWSimulator::rollBack(State &state){

    // TODO: roll back manipulators

    _cgraph->rollBack(state);
    // roll back bodies
    BOOST_FOREACH(RWBody *body, _bodies){
        body->rollBack(state);
    }
}


#ifdef RWSIMULATOR_DEBUG

void RWSimulator::internalStep(double dt, rw::kinematics::State& state){


#ifdef RWSIMULATOR_DEBUG
    std::cout << "*************************** UPDATE ****************************" << std::endl;

//    std::cout << "* --------------->UPDATE FORCES< " << std::endl;
//    std::cout << "* reset all forces and impulses" << std::endl;
#endif

    // ------------------ FORCE RESET --------------------------------
    // first reset all force contributions to zero
    BodyList::iterator body = _bodies.begin();
    for(;body != _bodies.end(); ++body ){
        (**body).reset();
    }

#ifdef RWSIMULATOR_DEBUG
//    std::cout << "* Add external forces " << std::endl;
#endif

    std::vector<BodyController*>::iterator manipulator = _manipulators.begin();
    for(;manipulator!=_manipulators.end(); ++manipulator ){
        (**manipulator).addForces( state , _time );
    }

    _contactGraph->updateContacts( state );

    std::vector< std::vector<ConstraintEdge*> > groups =  _contactGraph->computeGroups();

    //addContactForces(state, groups, _dt);

#ifdef RWSIMULATOR_DEBUG
//    std::cout << "* --------------->UPDATE POSITION< " << std::endl;
//    std::cout << "* Initiallly save state "<< std::endl;
#endif

    saveState( _dt, state );
    State savedState = state;
    bool penetrating = false;

#ifdef RWSIMULATOR_DEBUG
//    std::cout << "* PERFORMING COLLIOSION RESOLUTION" << std::endl;
#endif

    do {
        // now update velocities
        updateVelocity( _dt, state );

#ifdef RWSIMULATOR_DEBUG
//        std::cout << "* --------------->UPDATE COLLISION FORCES< " << std::endl;
//        std::cout << "* Add collision impulses" << std::endl;
#endif
        // Impulse solving
        addCollisionForces(state, groups, _dt);

        //addContactForces(state, groups, _dt);

        // *****************************************************************
        // update position state
#ifdef RWSIMULATOR_DEBUG
//        std::cout << "* update position " << std::endl;
#endif

        updatePosition( _dt, state );
        ContactGraph &cGraph = *_contactGraph;
        // *****************************************************************
        //std::cout << "Performing Broad phase update of contact graph" << std::endl;
#ifdef RWSIMULATOR_DEBUG
//        std::cout << "* Broad phase" << std::endl;
#endif

        cGraph.broadPhase( state, false);

#ifdef RWSIMULATOR_DEBUG
        std::cout << "* Narrow phasse" << std::endl;
#endif

        penetrating = cGraph.narrowPhase( state, false);

        if(penetrating){
            //std::cout << "* rollback to last stable state " << std::endl;
            state = savedState;
            rollBack( state );

            _dt = _dt/2;
#ifdef RWSIMULATOR_DEBUG
            std::cout << "* PENETRATING TRUE, ROLLBACK: " << _time << std::endl;
            std::cout << "* Reduce dt: " << _dt << std::endl;
#endif
        }
    } while(penetrating);

    for(body = _bodies.begin(); body != _bodies.end(); ++body ){
        if( (*body)->getNodeType() != ConstraintNode::Rigid )
            continue;
        std::cout << "VELOCITY OF BODY: " << (*body)->getFrame()->getName() << std::endl;
        std::cout << "LinVel: " <<((RigidBody*)(*body))->getLinVel() << std::endl;
        std::cout << "AngVel: " <<((RigidBody*)(*body))->getAngVel() << std::endl;
    }

    // update time
    _time += _dt;
    // calculate new timestep
    /*
    double dt = calcTimeStep();

    if( dt>_dt/4 + _dt ) {
        dt = _dt + _dt/4;
    }else if( dt<_dt - _dt/4 ){
        dt = _dt - _dt/4;
    }
    _dt = dt;*/
    if(_dt<0.001){
        _dt = 0.001;
    } else {
        _dt += 0.5*_dt;
    }
    if(_dt>0.03)
        _dt = 0.03;
#ifdef RWSIMULATOR_DEBUG
    std::cout << "* Adjust timestep to: " << _dt << std::endl;
    std::cout << "*************************** UPDATE END *************************" << std::endl;
#endif

}
#endif
