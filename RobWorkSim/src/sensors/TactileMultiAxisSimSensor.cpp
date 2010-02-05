/*
 * TactileMultiAxisSimSensor.hpp
 *
 *  Created on: 25-08-2008
 *      Author: jimali
 */

#include "TactileMultiAxisSimSensor.hpp"

using namespace rw::sensor;

TactileMultiAxisSimSensor::TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body):
    TactileMultiAxisSensor(name, &body->getBodyFrame())
{

}

rw::math::Transform3D<> TactileMultiAxisSimSensor::getTransform(){

}

rw::math::Vector3D<> TactileMultiAxisSimSensor::getForce(){

}

rw::math::Vector3D<> TactileMultiAxisSimSensor::getTorque(){

}

void TactileMultiAxisSimSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& cnormal,
               dynamics::Body *body)
{

}

void TactileMultiAxisSimSensor::addForce(const rw::math::Vector3D<>& point,
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& cnormal,
              dynamics::Body *body)
{

}
