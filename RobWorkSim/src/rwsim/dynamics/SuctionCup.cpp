#include "SuctionCup.hpp"

#include <rw/geometry/Geometry.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/CollisionModelInfo.hpp>

#include <string>
#include <iostream>

using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

SuctionCup::SuctionCup(Body* baseBody, const rw::math::Transform3D<>& bTb2, double radi, double height, double elasticity):
    _baseBody(baseBody), _radius(radi), _height(height), _elasticity(elasticity), _bTb2(bTb2)
{
    // we use only 4 bodies to represent the mouth of the suction cup
    Frame *_bframe = baseBody->getBodyFrame();
    _frames.push_back( new MovableFrame( _bframe->getName()+".m1" ) );
    _frames.push_back( new MovableFrame( _bframe->getName()+".m2" ) );
    _frames.push_back( new MovableFrame( _bframe->getName()+".m3" ) );
    _frames.push_back( new MovableFrame( _bframe->getName()+".m4" ) );

    std::stringstream sstr;
    sstr << "#Sphere " << radi/10 ;

    CollisionModelInfo cinfo(sstr.str(), "SmallSphere");
    BOOST_FOREACH(MovableFrame *f, _frames){
        CollisionModelInfo::set(std::vector<CollisionModelInfo>(1,cinfo), f);
        //f->getPropertyMap().add("CollisionInfo", "", cinfo);
    }

    BodyInfo info = _baseBody->getInfo();
    info.inertia = InertiaMatrix<>::makeSolidSphereInertia(0.01, radi/10);
    std::cout << "INERTIA: " << info.inertia << std::endl;
    info.mass = 0.01;
    info.masscenter = Vector3D<>(0,0,0);

    _bodies.push_back( new RigidBody(info, _frames[0], Geometry::makeSphere(radi/10) ) );
    _bodies.push_back( new RigidBody(info, _frames[1], Geometry::makeSphere(radi/10) ) );
    _bodies.push_back( new RigidBody(info, _frames[2], Geometry::makeSphere(radi/10) ) );
    _bodies.push_back( new RigidBody(info, _frames[3], Geometry::makeSphere(radi/10) ) );



    _sensors.push_back( ownedPtr( new BodyContactSensor(_bframe->getName()+".s1", _frames[0]) ));
    _sensors.push_back( ownedPtr( new BodyContactSensor(_bframe->getName()+".s2", _frames[1]) ));
    _sensors.push_back( ownedPtr( new BodyContactSensor(_bframe->getName()+".s3", _frames[2]) ));
    _sensors.push_back( ownedPtr( new BodyContactSensor(_bframe->getName()+".s4", _frames[3]) ));

    _bodyTransforms.push_back(Transform3D<>( _bTb2*Vector3D<>(0,-_radius, height) ) );
    _bodyTransforms.push_back(Transform3D<>( _bTb2*Vector3D<>(-_radius,0, height) ));
    _bodyTransforms.push_back(Transform3D<>( _bTb2*Vector3D<>(0,_radius, height) ));
    _bodyTransforms.push_back(Transform3D<>( _bTb2*Vector3D<>(_radius,0, height) ));

}

Body* SuctionCup::getBodyPart(){
    return _baseBody;
}

rw::kinematics::Frame* SuctionCup::getBodyFrame(){
    return _baseBody->getBodyFrame();
}

std::vector<RigidBody*>& SuctionCup::getBodyParts(){
    return _bodies;
}

std::vector<MovableFrame*>& SuctionCup::getFrameParts(){
    return _frames;
}

void SuctionCup::addToWorkCell(DynamicWorkCell::Ptr dwc){
    StateStructure::Ptr sstruct = dwc->getWorkcell()->getStateStructure();

    //sstruct->addFrame(getBodyFrame(), parent);
    //sstruct->addData(_baseBody);
    BOOST_FOREACH(Frame *frame, _frames){
        sstruct->addFrame(frame, getBodyFrame());
    }

    BOOST_FOREACH(Body *body, _bodies){
        sstruct->addData(body);
        dwc->addBody( body );
    }

    // now add contact sensing to the bodies
    BOOST_FOREACH(BodyContactSensor::Ptr sensor, _sensors){
        dwc->addSensor( sensor );
    }

    // update the default state such that the transforms of the movables is valid
    State state = sstruct->getDefaultState();
    for(size_t i=0;i<_frames.size();i++){
        _frames[i]->setTransform(_bodyTransforms[i], state);
    }

    sstruct->setDefaultState(state);


}
