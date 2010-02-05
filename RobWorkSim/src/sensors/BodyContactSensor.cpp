#include "BodyContactSensor.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::math;

BodyContactSensor::BodyContactSensor(const std::string& name,rw::kinematics::Frame* frame):
    Sensor(frame,name)
{

}

BodyContactSensor::~BodyContactSensor(){}

void BodyContactSensor::update(double dt, rw::kinematics::State& state){
    //if(_contactsTmp.size()>0)
     //   std::cout << "Nr contacts in update: " << _contactsTmp.size() << std::endl;
     _contacts = _contactsTmp;
     _contactsTmp.clear();

     // update aux variables
     _wTf = Kinematics::worldTframe( getFrame(), state);
     _fTw = inverse(_wTf);
}


void BodyContactSensor::reset(const rw::kinematics::State& state){

}

rw::sensor::Sensor* BodyContactSensor::getSensor(){
    return this;
}

void BodyContactSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& snormal,
               dynamics::Body *body)
{
    // TODO: NASTY NASTY HACK only for test at KTH
    //if( fabs( acos(dot( Vector3D<>(0,0,1), normalize(snormal) )))< 45*Deg2Rad )
    //    return;
    addForce(_fTw*point, _fTw.R()*force, _fTw.R()*snormal);
}

void BodyContactSensor::addForce(const rw::math::Vector3D<>& point,
                  const rw::math::Vector3D<>& force,
                  const rw::math::Vector3D<>& snormal,
                  dynamics::Body *body)
{
    _contactsTmp.push_back( Contact3D(point, snormal, force)  );
}
