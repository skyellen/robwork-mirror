#include "KinematicDevice.hpp"

#include <rw/common/macros.hpp>

KinematicDevice::KinematicDevice(
				dynamics::Body* base,
				const std::vector<dynamics::KinematicBody*> bodies,
                rw::models::Device *dev,
                rw::models::WorkCell* wc):
                    DynamicDevice(base,dev,wc),
                    _bodies(bodies),
                    _maxVel(dev->getVelocityLimits()),
                    _maxAcc(dev->getAccelerationLimits()),
                    _q( rw::math::Q::zero(dev->getDOF()) ),
                    _velQ( rw::math::Q::zero(dev->getDOF()) )
{

}

KinematicDevice::~KinematicDevice(){

}

const std::vector<dynamics::KinematicBody*>&
    KinematicDevice::getBodies(){

    return _bodies;
}


// parameters for velocity profile
void KinematicDevice::setMaxAcc(const rw::math::Q& acc){
    RW_ASSERT( acc.size()==_dev->getDOF() );
    // todo
    _maxAcc = acc;
}

rw::math::Q KinematicDevice::getMaxAcc(){
    return _maxAcc;
}

void KinematicDevice::setMaxVel(const rw::math::Q& vel){
    RW_ASSERT( vel.size()== _dev->getDOF() );
    // todo
    _maxVel = vel;
}

rw::math::Q KinematicDevice::getMaxVel(){
    return _maxVel;
}
