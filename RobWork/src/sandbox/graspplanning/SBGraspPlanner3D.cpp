#include "SBGraspPlanner3D.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/ResolvedRateSolver.hpp>
#include <rw/invkin/SimpleMultiSolver.hpp>
#include <rw/math/Math.hpp>
#include <boost/foreach.hpp>

#include <rw/geometry/Contour2DUtil.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/geometry/TriMesh.hpp>


#include "CG3IKSolver2D.hpp"
#include "CMDistCCPMeasure3D.hpp"
#include "ApproachMeasure3D.hpp"
#include "SimpleMeasure.hpp"

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rwlibs::proximitystrategies;
using namespace rw::graspplanning;

namespace {
    struct MyCollisionDetector {
        MyCollisionDetector(TreeDevice* hand, Frame *object, CollisionStrategy *strategy):
            _hand(hand),_obj(object),_strategy(strategy)
        {

        }

        std::vector<Frame*> query(State& state){
            std::vector<Frame*> result;
            Transform3D<> wTobj = Kinematics::worldTframe(_obj,state);
            BOOST_FOREACH( Frame *frame, _hand->frames()){
                RW_ASSERT(frame);
                Transform3D<> wTf = Kinematics::worldTframe(frame,state);

                if( _strategy->inCollision(frame,wTf,_obj,wTobj) ){
                    result.push_back(frame);
                    std::cout << frame->getName() << std::endl;
                    return result;
                }
            }
            return result;
        }

        bool query(std::vector<Joint*>& frames, int startIdx, State& state){
            Transform3D<> wTobj = Kinematics::worldTframe(_obj, state);
            for(size_t i=startIdx;i<frames.size();i++){
                Joint *frame = frames[i];
                Transform3D<> wTf = Kinematics::worldTframe(frame,state);

                if( _strategy->inCollision(frame,wTf,_obj,wTobj) ){
                    return true;
                }
            }
            return false;
        }

        TreeDevice *_hand;
        Frame *_obj;
        CollisionStrategy *_strategy;
    };

    RPY<> randRPY(){
        double roll = Math::ran(-Pi/2, Pi/2);
        double pitch = Math::ran(-Pi/2, Pi/2);
        double yaw = Math::ran(-Pi/2, Pi/2);
        return RPY<>(roll,pitch,yaw);
    }

    RPY<> randRPY(double rRange, double pRange, double yRange){
        double roll = Math::ran(-rRange, rRange);
        double pitch = Math::ran(-pRange, pRange);
        double yaw = Math::ran(-yRange, yRange);
        return RPY<>(roll,pitch,yaw);
    }

}

SBGraspPlanner3D::SBGraspPlanner3D(
        rw::models::TreeDevice *hand,
        std::vector<std::vector<rw::models::Joint*> >& fingers,
        QSamplerPtr preshapeSampler,
        const rw::kinematics::State& state):
            _hand(hand),
            _obj(NULL),
            _fingers(fingers),
            _preshapeSampler(preshapeSampler),
            _roll(Pi/2),
            _pitch(Pi/2),
            _yaw(Pi/2)

{
    _strategy = new ProximityStrategyPQP();
    _strategy->setFirstContact(true);

    for(size_t i=0; i<fingers.size(); i++){
        _fingerTipFrames.push_back(fingers[i].back());
        std::vector<std::pair<Q,Q> > bounds;
        for(size_t j=0;j<fingers[i].size();j++){
            bounds.push_back( fingers[i][j]->getBounds() );
        }
        _fingerBounds.push_back(bounds);
    }

    _endEffFrames = _hand->getEnds();

    /*
    int nrOfPreshapes = 10;
    std::vector<Q> preshapes(nrOfPreshapes);
    Q q(Q::zero(10));
    q(0) = -1.2;
    q(4) = -1.2;
    q(7) = -1.2;

    for(int i=0;i<nrOfPreshapes; i++){
        // vary q(3)
        q(3) = Math::ran(0.001, 1.56);
        preshapes[i] = q;
    }

    QSamplerPtr sampler = QSampler::makeFinite(preshapes);
     */

}

SBGraspPlanner3D::~SBGraspPlanner3D(){
    delete _strategy;
}

void SBGraspPlanner3D::reset(rw::kinematics::MovableFrame* obj,const rw::kinematics::State& state){
    RW_ASSERT(obj);

    _obj = obj;
    std::vector<GeometryPtr> geoms = GeometryFactory::loadCollisionGeometry(*_obj);

    if(geoms.size()==0)
        RW_THROW("No geometry associated with frame: " << _obj->getName());

    //float mu = _obj->getPropertyMap().get<float>("ColoumbFrictionMU",0.5);
    // for now we assume that all contacts are described relative to world.
    Transform3D<> wTo = Kinematics::worldTframe(_obj, state);

    // setup the grasp quality measures
    _objCM = GeometryUtil::estimateCOG(geoms);
    _objRadii = GeometryUtil::calcMaxDist(geoms, _objCM);
    std::cout << "cm: " << _objCM << std::endl;
    std::cout << "objRadi: " << _objRadii << std::endl;
}

std::vector<SBGraspPlanner3D::GraspResult> SBGraspPlanner3D::query(const rw::kinematics::State& initstate, int maxNrOfQs){
    RW_ASSERT(_obj);
    State state = initstate;
    std::cout << "Finger tip frames: \n-";
    BOOST_FOREACH(Frame *frame, _fingerTipFrames){
        std::cout << frame->getName() << "\n-";
    }

    std::cout << "End effector frames: \n-";
    BOOST_FOREACH(Frame *frame, _endEffFrames){
        std::cout << frame->getName() << "\n-";
    }

    std::vector<GraspResult> graspresult;

    MyCollisionDetector detector(_hand, _obj, _strategy);

    double stepVel = 0.4;

    int NR_OF_TRIES = maxNrOfQs;
    _progressRange = std::make_pair(0, NR_OF_TRIES-1);
    _progress = 0;


    //******************************************

    // now we do this plenty of times...
    for(int tries=0;tries<NR_OF_TRIES;tries++){
        _progress = tries;
        // set the hand in a one of the preshapes
        std::cout << "sample" << std::endl;
        if(_preshapeSampler->empty())
            break;
        Q preshapeQ = _preshapeSampler->sample();
        _hand->setQ( preshapeQ, state);
        std::cout << "s" << std::endl;
        // generate a random object pose in the vecinity of the hand
        Vector3D<> pos(Math::ran(-_objRadii/2,_objRadii/2),
                       Math::ran(-_objRadii/2,_objRadii/2),
                       Math::ran(-_objRadii/4,_objRadii/4));

        Rotation3D<> rotDiff = randRPY(_roll,_pitch,_yaw).toRotation3D();

        // the approach vector of the object must align with the z-axis of the base
        Transform3D<> hbTo =  _bTo; //Kinematics::frameTframe(_hand->getBase(), _obj, state);
        hbTo.P() += pos;
        hbTo.R() = rotDiff*hbTo.R();

        // hbTo = hbTw * wTp * pTo;
        //Vector3D<> hbPapproach = hbTo.R()*_approach;

        // now calculate the rotation 3D that will rotate hbPapproach into z-axis
        //EAA<> eaa(hbPapproach, Vector3D<>(0,0,1));
        //
        //Rotation3D<> rot = eaa.toRotation3D()*hbTo.R();

        //Rotation3D<> rotDiff = randRPY(Pi/2, _thresAngle,_thresAngle).toRotation3D();
        //Transform3D<> hTobj(pos, rotDiff*rot);
        // remember to compensate for the center of mass position
        Transform3D<> hTobj = hbTo;
        hTobj.P() -= hTobj.R()*_objCM;
        // relative to the hand

        Transform3D<> hTobjP = Kinematics::frameTframe(_hand->getBase(), _obj->getParent(), state);
        _obj->setTransform( inverse(hTobjP)*hTobj, state);

        std::cout << hTobj.P() << std::endl;
        std::cout << preshapeQ << std::endl;

        // next close all fingers until they are colliding with the object
        std::vector<Frame*> colResult = detector.query(state);
        if(colResult.size()>0){
            std::cout << "START CFG IN COLLISION - RETRY" << std::endl;
            continue;
        }
        std::cout << "coldect" << std::endl;

        //int nrOfCloseJoints=0;
        int outOfBounds=0;
        const double smallAngle = 0.1;
        // for each finger close it around the object
        for(size_t fingerIdx=0;fingerIdx<_fingers.size();fingerIdx++){
            std::vector<Joint*>& finger = _fingers[fingerIdx];
            // we change only one joint at the time
            double jointRes = 1.0;
            do {
                for(size_t jointIdx=0;jointIdx<finger.size();jointIdx++){
                    Joint *joint = finger[jointIdx];
                    double q = *(joint->getData(state));
                    q += stepVel * jointRes;
                    finger[jointIdx]->setData(state, &q);
                    bool res = detector.query(finger, jointIdx, state);
                    if( res ){
                        q -= stepVel*jointRes;
                        finger[jointIdx]->setData(state, &q);
                        jointRes *= 0.5;
                    }
                    std::pair<Q,Q> &bound = _fingerBounds[fingerIdx][jointIdx];
                    if( q < bound.first(0) || bound.second(0)< q ){
                        outOfBounds = true;
                    }
                }
            } while(jointRes>smallAngle && !outOfBounds);
        }

        if( outOfBounds )
            continue;

        // now if we are not out of bounds then we have a potential grasp
        // though we need to calculate the contacts and stuff
        // for this we use the distance strategy

        rw::proximity::DistanceResult result;
        Grasp3D g3d(3);
        bool dontSave = false;
        Transform3D<> wTh = Kinematics::worldTframe(_hand->getBase(),state);
        for(size_t i=0;i<_fingerTipFrames.size();i++){
             Frame *frame = _fingerTipFrames[i];
             Transform3D<> wTf = Kinematics::worldTframe(frame, state);
             bool res = _strategy->distance(result, frame, wTf, _obj, wTh*hTobj);
             if(!res  ){
                 dontSave = true;
                 break;
             }

             std::cout << "Distance: " << result.distance << " "
                       //<< MetricUtil::dist2(wTh*(hTobj*result.p2),wTf*result.p1) << " "
                       //<< MetricUtil::dist2(result.p2,result.p1)
                       << "\n";

             // for now we estimate the contact normal as the shortest vector
             // between the two objects. Perhaps not entirely true... consider the surface normal
             Vector3D<> n = (wTh*hTobj)*result.p2-wTf*result.p1;

             // next we extract the approach vector which is just the heading of the z-axis
             // in the end effector
             g3d.approach[i] = Kinematics::worldTframe(_endEffFrames[i],state).R()*Vector3D<>(0,0,1);

             g3d.contacts[i].p = wTh*(hTobj*result.p2); // describe all in world
             g3d.contacts[i].n = normalize(n);
             g3d.contacts[i]._faceIdx = result.idx2;
        }

        //Transform3D<> wThb = Kinematics::worldTframe(_hand->getBase(),state);
        //Vector3D<> wPapproach = wThb.R()*hbPapproach;


        /*
        // check angles between object approach and finger approach. The must
        // be larger than 80 deg
        Transform3D<> wThb = Kinematics::worldTframe(_hand->getBase(),state);
        Vector3D<> wPapproach = wThb.R()*hbPapproach;
        for(int i=0;i<g3d.approach.size();i++){
            double angle = acos( dot(normalize(wPapproach),normalize(g3d.approach[i]) ) );
            if( angle<70*Deg2Rad || angle>110*Deg2Rad ){
                std::cout << "Angle too small or large: " << angle*Rad2Deg << "Deg\n";
                dontSave = true;
                break;
            }
        }
*/

        // well only save the grasp if distance results where valid
        if(!dontSave){
            Transform3D<> wTo = Kinematics::worldTframe(_obj, state);
            CMDistCCPMeasure3D cmccp( wTo*_objCM, _objRadii);
            ApproachMeasure3D appM(45*Deg2Rad);

            g3d.quality = cmccp.quality(g3d);
            g3d.quality += appM.quality(g3d);
            g3d.quality /= 2;

            GraspResult gres(g3d, _hand->getQ(state), inverse( hTobj ));
            graspresult.push_back(gres);
        }
    }
    return graspresult;
}
