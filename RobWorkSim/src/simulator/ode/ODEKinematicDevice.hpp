/*
 * ODEVelocityDevice.hpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#ifndef ODEKINEMATICDEVICE_HPP_
#define ODEKINEMATICDEVICE_HPP_

#include <ode/ode.h>

#include <vector>

#include <rw/math/Q.hpp>
#include <dynamics/KinematicDevice.hpp>

#include "ODEJoint.hpp"
#include "ODEDevice.hpp"

/**
 * @brief A bridge between the RW KinematicDevice and kinematicly controlled
 * ODE dBodies.
 */
class ODEKinematicDevice: public ODEDevice {
public:

    ODEKinematicDevice(KinematicDevice *rdev, const std::vector<dBodyID>& kbodies);

    virtual ~ODEKinematicDevice();

    void reset(rw::kinematics::State& state);

    /**
     * @brief
     * @param dt
     * @param state
     */
    void update(double dt, rw::kinematics::State& state);

    void postUpdate(rw::kinematics::State& state);

private:
    KinematicDevice *_kdev;
    rw::math::Q _maxVel;
    std::vector<dBodyID> _kbodies;
};

#endif /* ODEVELOCITYDEVICE_HPP_ */
