/*
 * IKSoftCSolver.cpp
 *
 *  Created on: 27-05-2009
 *      Author: jimali
 */

#include "IKSoftCSolver.hpp"

#include <rw/kinematics/Kinematics.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/MetricUtil.hpp>

#include <rw/models/JointDevice.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/common/StringUtil.hpp>

#include <boost/foreach.hpp>

#include <rw/common/TimerUtil.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;

namespace {
    rw::math::Transform3D<> getODEBodyT3D(dBodyID bodyId){
        const dReal *v = dBodyGetPosition(bodyId);
        const dReal *q = dBodyGetQuaternion(bodyId);

        Vector3D<> pos(v[0],v[1],v[2]);
        Quaternion<> quat(q[1],q[2],q[3],q[0]);
        return Transform3D<>(pos,quat);
    }

    void toODERotation(const rw::math::Rotation3D<>& rwR, dMatrix3 R){
        R[0] = rwR(0,0);
        R[1] = rwR(0,1);
        R[2] = rwR(0,2);
        R[4] = rwR(1,0);
        R[5] = rwR(1,1);
        R[6] = rwR(1,2);
        R[8] = rwR(2,0);
        R[9] = rwR(2,1);
        R[10] = rwR(2,2);
    }


    void setODEBodyT3D(dBodyID bodyId, const rw::math::Transform3D<>& t3d){
        dBodySetPosition(bodyId, t3d.P()[0], t3d.P()[1], t3d.P()[2] );
        rw::math::Quaternion<> rwQuat( t3d.R() );
        dMatrix3 R;
        toODERotation(t3d.R(),R);
        dBodySetRotation(bodyId, R);
    }


    int getParent(Frame *frame, const std::vector<Frame*>& frames, const State& state){
        Frame *parent = frame->getParent(state);
        while(parent!=NULL){
            for(size_t i=0; i<frames.size(); i++){
                if(frames[i]==parent)
                    return i;
            }
            parent = parent->getParent(state);
        }
        RW_THROW("INVALID INDEX!");
        return -1;
    }

    void createODEConstraintSolver(
         const Frame *devbase,
         JointDevice *device,
         const std::vector<Frame*>& ends,
         const State& state,
         IKSoftCSolver::InvKinState& ikState,
         bool isBaseMovable)
    {
         // we use hashspace here because devices typically have
         // relatively few bodies
        State initState = state;
        ikState.endeff = ends;
        // add kinematic constraints from base to joint1, joint1 to joint2 and so forth

        // TODO: create a rigid body for the base
        {
            Frame *base = device->getBase();
            dBodyID bodyId = 0; // defaults to fixed or world
            if(isBaseMovable){
                bodyId = dBodyCreate(ikState.worldId);
                Transform3D<> wTb = Kinematics::frameTframe(devbase, base, state);
                setODEBodyT3D(bodyId, wTb);
                dMass mass;
                dMassSetSphereTotal(&mass, 0.2, 0.05);
                dBodySetMass(bodyId, &mass);
            }

            ikState.allbodies.push_back(bodyId);
            ikState.frames.push_back(base);
            ikState.bodyMap[base] = bodyId;
            ikState.bodyToFrameMap[bodyId] = base;

        }

        Q low = device->getBounds().first;
        Q high = device->getBounds().second;
        std::map<Frame*, std::pair<double,double> > jointBoundMap;
        BOOST_FOREACH(Joint *joint, device->getJoints()){
            if(joint->getDOF()==0)
                continue;
            std::pair<Q,Q> bound = joint->getBounds();
            jointBoundMap[joint] = std::pair<double,double>(bound.first[0],bound.second[0]);
        }
/*
        for(size_t i=0;i<device->getDOF();i++){
            std::pair<double,double> bound(low[i],high[i]);
            jointBoundMap[device->getActiveJoint(i)] = bound;
        }
*/
        // locate any passive joints that we are not able to detect in active joints
        //std::cout << "NR ENDS: " << ikState.endeff.size() << std::endl;
        BOOST_FOREACH(Frame *end, ikState.endeff){
            //std::cout << "END: " << end->getName() << std::endl;
            std::vector<Frame*> frames = Kinematics::parentToChildChain(device->getBase(), end, state);
            //std::cout << "--- Frames : " << frames.size() << std::endl;
            BOOST_FOREACH(Frame *frame, frames){
                //std::cout << "frame: " << frame->getName() << std::endl;
                //if( dynamic_cast<PassiveRevoluteFrame*>(frame) ){
                //    std::cout << "---- PassiveRevoluteFrame" << std::endl;
                //}

                if( dynamic_cast<DependentRevoluteJoint*>(frame) ||
                        dynamic_cast<RevoluteJoint*>(frame) ||
                        dynamic_cast<PrismaticJoint*>(frame) ){

                    if(ikState.bodyMap.find(frame)==ikState.bodyMap.end()){
                        dBodyID bodyId = dBodyCreate(ikState.worldId);
                        Transform3D<> wTb = Kinematics::frameTframe(devbase, frame, state);
                        setODEBodyT3D(bodyId, wTb);
                        dMass mass;
                        dMassSetSphereTotal(&mass, 0.2, 0.05);
                        dBodySetMass(bodyId, &mass);

                        ikState.frames.push_back(frame);
                        ikState.bodyMap[frame] = bodyId;
                        ikState.bodyToFrameMap[bodyId] = frame;
                        ikState.allbodies.push_back(bodyId);
                    }
                }
            }
        }

        for(size_t bodyIdx=0;bodyIdx<ikState.frames.size();bodyIdx++){
            Frame *joint = ikState.frames[bodyIdx];
            dBodyID jBody = ikState.allbodies[bodyIdx];
            Transform3D<> wTjoint = Kinematics::frameTframe(devbase,joint,initState);
            if(RevoluteJoint *rwjoint = dynamic_cast<RevoluteJoint*>(joint)){
                // find the parent joint
                int i = getParent(joint, ikState.frames, state);
                Frame *parent = ikState.frames[i];
                //dBodyID pBody = ikState.bodyMap[parent]
                dBodyID pBody = ikState.allbodies[i];
                const double qinit = rwjoint->getData(initState)[0];

                // we rotate around the z-axis
                Vector3D<> haxis = wTjoint.R() * Vector3D<>(0,0,1);
                Vector3D<> hpos = wTjoint.P();

                dJointID hinge = dJointCreateHinge (ikState.worldId, 0);
                dJointAttach(hinge, jBody, pBody);
                dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
                dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

/*
                dJointID motor = dJointCreateAMotor (ikState.worldId, 0);
                dJointAttach(motor, jBody, pBody);
                dJointSetAMotorNumAxes(motor, 1);
                dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                dJointSetAMotorAngle(motor, 0, qinit);
                dJointSetAMotorParam(motor,dParamFMax, 0.001 );
                dJointSetAMotorParam(motor,dParamVel,0);
*/
                if( jointBoundMap.find(rwjoint)!=jointBoundMap.end() ){
                    std::pair<double,double> bound = jointBoundMap[rwjoint];
                    dJointSetHingeParam (hinge, dParamLoStop, bound.first);
                    dJointSetHingeParam (hinge, dParamHiStop, bound.second);
                }

                ikState.jointMap[rwjoint] = hinge;
                //ikState.motorMap[rwjoint] = motor;
                ikState.joints.push_back(rwjoint);

                //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                //dJointSetAMotorParam(Amotor,dParamHiStop,0);
            } else if( dynamic_cast<PrismaticJoint*>(joint) ){
                std::cout << "PRISMATICJOINT" << std::endl;
                PrismaticJoint* pjoint = dynamic_cast<PrismaticJoint*>(joint);
                int i = getParent(joint, ikState.frames, state);
                Frame *parent = ikState.frames[i];
                dBodyID pBody = ikState.allbodies[i];
                const double qinit = pjoint->getData(initState)[0];

                // we rotate around the z-axis
                Vector3D<> haxis = wTjoint.R() * Vector3D<>(0,0,-1);
                Vector3D<> hpos = wTjoint.P();

                dJointID slider = dJointCreateSlider (ikState.worldId, 0);
                dJointAttach(slider, jBody, pBody);
                dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));
                //dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

                if( jointBoundMap.find(pjoint)!=jointBoundMap.end() ){
                    std::pair<double,double> bound = jointBoundMap[pjoint];
                    dJointSetSliderParam (slider, dParamLoStop, bound.first);
                    dJointSetSliderParam (slider, dParamHiStop, bound.second);
                }

                ikState.jointMap[pjoint] = slider;
                //ikState.motorMap[pjoint] = motor;
                ikState.joints.push_back(pjoint);

            } else if( DependentRevoluteJoint *pframe = dynamic_cast<DependentRevoluteJoint*>(joint) ){
                //std::cout << "pass" << std::endl;
                int i = getParent(joint, ikState.frames, state);
                Frame *parent = ikState.frames[i];
                //dBodyID pBody = ikState.bodyMap[parent]
                dBodyID pBody = ikState.allbodies[i];
                const double qinit = pframe->calcQ(initState);


                // we rotate around the z-axis
                Vector3D<> haxis = wTjoint.R() * Vector3D<>(0,0,1);
                Vector3D<> hpos = wTjoint.P();

                dJointID hinge = dJointCreateHinge (ikState.worldId, 0);
                dJointAttach(hinge, jBody, pBody);
                dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
                dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

                dJointID motor = dJointCreateAMotor (ikState.worldId, 0);
                dJointAttach(motor, jBody, pBody);
                dJointSetAMotorNumAxes(motor, 1);
                dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                dJointSetAMotorAngle(motor,0, qinit);
                dJointSetAMotorParam(motor,dParamFMax, 0.1 );
                dJointSetAMotorParam(motor,dParamVel,0);

                ikState.jointMap[pframe] = hinge;
                ikState.motorMap[pframe] = motor;
                ikState.joints.push_back(pframe);
                ikState.depJoints.push_back(pframe);
            } else {

            }
        }

        std::vector<dBodyID> endBodies;

        BOOST_FOREACH(Frame *end, ikState.endeff){
            int i = getParent(end, ikState.frames, state);
            Frame *parent = ikState.frames[i];
            dBodyID pBody = ikState.allbodies[i];
            ikState.endJBodies.push_back(pBody);
            ikState.endJoints.push_back(parent);
        }
    }

}
bool initialized = false;

namespace {
    void EmptyMessageFunction (int errnum, const char *msg, va_list ap){

    }

    void initODE(){
        //if(!initialized){
            dInitODE();
            initialized = true;
            dSetErrorHandler(EmptyMessageFunction);
            dSetDebugHandler(EmptyMessageFunction);
            dSetMessageHandler(EmptyMessageFunction);
        //}
    }
}

IKSoftCSolver::IKSoftCSolver(rw::models::SerialDevice* sdev,
                             rw::models::TreeDevice* device,
                             const std::vector<rw::kinematics::Frame*>& foi,
                             const rw::kinematics::State& state)
:
IterativeMultiIK(foi.size()),
_dt(0.05),
_foi(foi),
_arm(sdev),
_device(device)
{
    initODE();
    _ikState.worldId = dWorldCreate();
    _ikState.spaceId = dHashSpaceCreate(0);
    dWorldSetGravity ( _ikState.worldId, 0, 0, 0 );
    dWorldSetContactMaxCorrectingVel (_ikState.worldId, 1);
    createODEConstraintSolver(sdev->getBase(), device, _foi, state, _ikState, true);
}

IKSoftCSolver::IKSoftCSolver(rw::models::TreeDevice* device,
                             const rw::kinematics::State& state)
:
IterativeMultiIK(device->getEnds().size()),
_dt(0.05),
_foi(device->getEnds()),
_arm(NULL),
_device(device),
_returnBestFit(false)
{
    initODE();
    _ikState.worldId = dWorldCreate();
    _ikState.spaceId = dHashSpaceCreate(0);
    dWorldSetGravity ( _ikState.worldId, 0, 0, 0 );

    createODEConstraintSolver(device->getBase(), device, _foi, state, _ikState, true);
}

IKSoftCSolver::IKSoftCSolver(rw::models::JointDevice* device,
                             const std::vector<rw::kinematics::Frame*>& foi,
                             const rw::kinematics::State& state)
    :
    IterativeMultiIK(foi.size()),
    _dt(0.05),
    _foi(foi),
    _arm(NULL),
    _device(device),
    _returnBestFit(false)
{
    initODE();
    _ikState.worldId = dWorldCreate();
    _ikState.spaceId = dHashSpaceCreate(0);
    dWorldSetGravity ( _ikState.worldId, 0, 0, 0 );
    createODEConstraintSolver(device->getBase(), device, _foi, state, _ikState, true);
}

IKSoftCSolver::~IKSoftCSolver(){
    dCloseODE();
}

namespace {
    void reset(const Frame *base, IKSoftCSolver::InvKinState &ikstate, const State& state){
        // reset the body positions
        BOOST_FOREACH(dBodyID body, ikstate.allbodies){
            if(body==0)
                continue;
            const Frame *frame = ikstate.bodyToFrameMap[body];
            Transform3D<> baseTb = Kinematics::frameTframe(base, frame, state);
            setODEBodyT3D(body, baseTb);
            dBodyEnable( body );
            dBodySetAngularVel( body, 0, 0, 0 );
            dBodySetLinearVel( body, 0, 0, 0 );
            dBodySetForce(body, 0, 0, 0);
            dBodySetTorque (body, 0, 0, 0);

        }
    }

    std::vector<dJointID> addConstraints(
                                         const std::vector<rw::math::Transform3D<> >& baseTend,
                                         IKSoftCSolver::InvKinState &ikstate,
                                         const State& state)
    {
        std::vector<dJointID> constraints;

        // we cannot set the softness of a fixed joint..
        for(size_t i=0;i<baseTend.size();i++){
            dBodyID endBody = ikstate.endJBodies[i];

            Transform3D<> bt3d = getODEBodyT3D(endBody);

            // set the body transform
            Transform3D<> endToJend = Kinematics::frameTframe(ikstate.endeff[i],ikstate.endJoints[i],state);
            setODEBodyT3D(endBody, baseTend[i]*endToJend);

            dJointID fixedJoint = dJointCreateFixed(ikstate.worldId, 0);
            dJointAttach(fixedJoint, endBody, 0);
            dJointSetFixed(fixedJoint);

            setODEBodyT3D(endBody, bt3d);

            constraints.push_back(fixedJoint);
        }
        /*
        // instead we try with sliders
        for(size_t i=0;i<baseTend.size();i++){
            dBodyID endBody = ikstate.endJBodies[i];

            // set the body transform
            Transform3D<> endToJend = Kinematics::frameTframe(ikstate.endeff[i],ikstate.endJoints[i],state);
            Transform3D<> bt3d = getODEBodyT3D(endBody);
            setODEBodyT3D(endBody, baseTend[i]*endToJend);

            dJointID slider = dJointCreateSlider(ikstate.worldId, 0);
            Vector3D<> zaxis = (baseTend[i]*endToJend).R()*Vector3D<>::z();
            dJointAttach(slider, endBody, 0);
            dJointSetSliderAxis (slider, zaxis[0], zaxis[1], zaxis[2]);

            dJointSetSliderParam(slider, dParamLoStop, -0.01);
            dJointSetSliderParam(slider, dParamHiStop,  0.001);
            //dJointSetSliderParam(slider, dParamFudgeFactor,  0.2);
            //dJointSetSliderParam(slider, dParamCFM,  0.04);
            //dJointSetSliderParam(slider, dParamStopCFM,  0.04);
            //dJointSetSliderParam(slider, dParamStopERP,  0.08);

            setODEBodyT3D(endBody, bt3d);

            constraints.push_back(slider);
        }
*/

        return constraints;
    }


    bool isResting(IKSoftCSolver::InvKinState &ikstate){

        const double lthres = 0.01*0.01;
        const double athres = 0.001*0.001;
        /*
        BOOST_FOREACH(dBodyID body, ikstate.allbodies){
            if(body==0)
                continue;
            std::cout << getODEBodyT3D(body).P() << "\n";
        }
        */
        // check velocity of all bodies
        BOOST_FOREACH(dBodyID body, ikstate.allbodies){
            if(body==0)
                continue;
            const dReal *lvel = dBodyGetLinearVel(body);
            if( lthres<lvel[0]*lvel[0]+lvel[1]*lvel[1]+lvel[2]*lvel[2] ){
                //std::cout << lvel[0] << " " << lvel[1] << " " << lvel[2] << std::endl;
                return false;
            }

            const dReal *avel = dBodyGetAngularVel(body);
            if( athres<avel[0]*avel[0]+avel[1]*avel[1]+avel[2]*avel[2] ){
                //std::cout << avel[0] << " " << avel[1] << " " << avel[2] << std::endl;
                return false;
            }

        }
        return true;
    }
}

namespace {
    bool performLoop(const double dt, IKSoftCSolver::InvKinState &ikState, State &state, int maxIterations, Frame *base){
        int maxIter2 = maxIterations-maxIterations/4;
        int minIterations = maxIterations-maxIterations/8;
        do {
            //dBodyID baseId = ikState.bodyMap[base];
            //std::cout << "Hand Base: " << getODEBodyT3D(baseId).P() << std::endl;

            if(maxIterations>maxIter2){
                dWorldQuickStep(ikState.worldId, dt);
            } else {
                //dWorldQuickStep(ikState.worldId, dt);
                dWorldStep(ikState.worldId, dt);
            }

            // make sure to update dependent joints
            BOOST_FOREACH(Frame *frame, ikState.depJoints){
                if(DependentRevoluteJoint* pframe = dynamic_cast<DependentRevoluteJoint*>(frame)){
                    dJointID motor = ikState.motorMap[pframe];
                    dJointID joint = ikState.jointMap[pframe];
                    // read back angle pos of
                    dJointID owner = ikState.jointMap[&pframe->getOwner()];
                    double angle_owner = dJointGetHingeAngle( owner );
                    double scale = pframe->getScale();
                    double offset = pframe->getOffset();
                    double angle_dest = scale * angle_owner + offset;
                    double angle = dJointGetHingeAngle( joint );
                    double desVel = (angle_dest-angle)/dt;
                    dJointSetAMotorParam (motor,  dParamVel, desVel);
                    //std::cout << "desVel " << desVel << std::endl;
                }
            }
        } while( ((maxIterations>minIterations) || !isResting(ikState)) && (--maxIterations >0) );
        //std::cout << maxIterations << "\n";
        // now read joint angles back
        BOOST_FOREACH(Frame *frame, ikState.joints){
            if( dynamic_cast<RevoluteJoint*>(frame) ){
                RevoluteJoint *joint = dynamic_cast<RevoluteJoint*>(frame);
                dJointID odeJoint = ikState.jointMap[joint];
                double angle = dJointGetHingeAngle( odeJoint );
                joint->setData(state, &angle);
            } else if( dynamic_cast<PrismaticJoint*>(frame) ){
                PrismaticJoint *joint = dynamic_cast<PrismaticJoint*>(frame);
                dJointID odeJoint = ikState.jointMap[joint];
                double pos = dJointGetSliderPosition( odeJoint );
                joint->setData(state, &pos);
            }
        }
        return isResting(ikState);
    }
}



std::vector<rw::math::Q> IKSoftCSolver::solve(const std::vector<
        rw::math::Transform3D<> >& baseTend, const rw::kinematics::State& initState) const
{
    RW_ASSERT(baseTend.size()==_ikState.endeff.size());
    //std::cout << "Solve arm!" << std::cout;

    //std::cout << "SOLVE: hand base: " << Kinematics::frameTframe(_arm->getBase(), _device->getBase(), initState).P() << std::endl;

    State state = initState;

    Frame *base = _device->getBase();
    if(_arm!=NULL)
        base=_arm->getBase();

    reset(base, _ikState, state);

    // add constraints to the end effectors
    std::vector<dJointID> constraints = addConstraints(baseTend, _ikState, state);
    //std::cout << "destroy joints" << std::cout;

    int maxIterations = 400;
    bool sol = performLoop(_dt, _ikState, state, maxIterations, _device->getBase());

    // remove all constraints
    BOOST_FOREACH(dJointID joint, constraints){
        dJointDestroy(joint);
    }

    std::vector<Q> result;

    if(!sol)
        return result;

    if (_arm != NULL) {
        // we calculate a wrist pose that is an average of the baseTend poses
        dBodyID baseId = _ikState.bodyMap[_device->getBase()];
        Transform3D<> bTse = Kinematics::frameTframe(_device->getBase(), _arm->getEnd(), state);
        Transform3D<> sbTse = getODEBodyT3D(baseId)*bTse;
        //std::cout << "hand base after invkin: " << Kinematics::frameTframe(_arm->getBase(), _device->getBase(), state).P() << std::endl;

        rw::invkin::JacobianIKSolver solver(_arm, state);
        std::vector<Q> res = solver.solve( sbTse, state );
        if(res.size()==0){
            //std::cout << "NO ARM SOLUTION" << std::endl;
            return result;
        }
        _arm->setQ(res[0], state);
        //std::cout << "hand base after invkin: " << Kinematics::frameTframe(_arm->getBase(), _device->getBase(), state).P() << std::endl;
    }
    dBodyID baseId = _ikState.bodyMap[_device->getBase()];
    _handBaseT3d = getODEBodyT3D(baseId);
    // check if result is actually close to the wanted solution
    const double distThres = 0.01; // 4mm
    if(!_returnBestFit){

        bool badInvKin=false;
        for(size_t i=0;i<baseTend.size();i++){
            const Transform3D<>& bTed = baseTend[i];
            const Transform3D<>& bTe  = _handBaseT3d*Kinematics::frameTframe(_device->getBase(), _ikState.endeff[i], state);
            if( MetricUtil::dist2(bTe.P(),bTed.P())>distThres ){
                //std::cout << MetricUtil::dist2(bTe.P(),bTed.P()) << " , ";
                badInvKin=true;
            }
        }
        //std::cout << "\n";
        if(badInvKin)
            return result;
    }

    if (_arm != NULL) {
        Q armQ = _arm->getQ(state);
        result.push_back( concat( armQ, _device->getQ(state)) );
    } else {
        dBodyID baseId = _ikState.bodyMap[_device->getBase()];
        _handBaseT3d = getODEBodyT3D(baseId);
        result.push_back( _device->getQ(state) );
    }
    return result;
}
