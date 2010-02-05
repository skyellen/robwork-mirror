#ifndef RIGIDDEVICE_HPP_
#define RIGIDDEVICE_HPP_

#include <rw/math/Q.hpp>
#include <rw/math/Math.hpp>

#include "DynamicDevice.hpp"
#include "RigidJoint.hpp"

/**
 *
 */
class RigidDevice : public DynamicDevice {

public:

    /**
     *
     * @param bodies
     * @param dev
     * @param wc
     * @return
     */
    RigidDevice(dynamics::Body* base,
				const std::vector<dynamics::RigidJoint*>& bodies,
                rw::models::Device *dev,
                rw::models::WorkCell* wc):
        DynamicDevice(base,dev,wc),
        _vel( rw::math::Q::zero(dev->getDOF()) ),
        _force( rw::math::Q::zero(dev->getDOF()) ),
        _actualVel( rw::math::Q::zero(dev->getDOF()) ),
        _bodies(bodies)
    {

    }

    /**
     *
     * @return
     */
    virtual ~RigidDevice(){};

    /**
     *
     * @param force
     */
    void setForceLimit(const rw::math::Q& force){
        _force = force;
    }

    /**
     *
     * @return
     */
    rw::math::Q getForceLimit(){
        return _force;
    }

    rw::math::Q getVelocity(const rw::kinematics::State& state){
        return _vel;
    }

    void setVelocity(const rw::math::Q& vel, const rw::kinematics::State& state){
        rw::math::Q velLimit = getModel().getVelocityLimits();
       // std::cout  << "Vel limits: " << velLimit <<  std::endl;
       // std::cout  << "Before clamp: " << vel << std::endl;
        _vel = rw::math::Math::clampQ(vel, -velLimit, velLimit);
       // std::cout  << "after  clamp: " << _vel << std::endl;
    }

    const std::vector<dynamics::RigidJoint*>& getBodies(){
        return _bodies;
    }

    void setActualVelocity(const rw::math::Q& vel, const rw::kinematics::State& state){
        RW_ASSERT(vel.size()==_actualVel.size());
        _actualVel = vel;
    }

    rw::math::Q getActualVelocity(const rw::kinematics::State& state){
        return _actualVel;
    }
private:
    rw::math::Q _vel, _actualVel;
    rw::math::Q _force;
    std::vector<dynamics::RigidJoint*> _bodies;
};



#endif /*RIGIDDEVICE_HPP_*/
