/*
 * ODEVelocityDevice.hpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#ifndef ODEVELOCITYDEVICE_HPP_
#define ODEVELOCITYDEVICE_HPP_

#include <ode/ode.h>

#include <vector>

#include "ODEJoint.hpp"
#include <rw/math/Q.hpp>
#include <dynamics/RigidDevice.hpp>
#include "ODEJoint.hpp"
#include "ODEDevice.hpp"

class ODEVelocityDevice: public ODEDevice {
public:

    /**
     * @brief constructor
     */
    ODEVelocityDevice(
        RigidDevice *rdev,
        std::vector<ODEJoint*> odejoints,
        rw::math::Q maxForce);

    /**
     * @brief destructor
     */
    virtual ~ODEVelocityDevice();

    /**
     * @copydoc ODEDevice::reset
     */
    void reset(rw::kinematics::State& state);

    /**
     * @copydoc ODEDevice::update
     */
    void update(double dt, rw::kinematics::State& state);

    /**
     * @copydoc ODEDevice::postUpdate
     */
    void postUpdate(rw::kinematics::State& state);

    /**
     *
     * @param rdev
     * @param base
     * @return
     */
    static ODEVelocityDevice* makeDevice(RigidDevice *rdev,
										dBodyID base,
										dSpaceID space,
										dWorldID worldId);

private:
    RigidDevice *_rdev;
    std::vector<ODEJoint*> _odeJoints;
    rw::math::Q _maxForce;
};

#endif /* ODEVELOCITYDEVICE_HPP_ */
