/*
 * ODEDevice.hpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#ifndef ODEDEVICE_HPP_
#define ODEDEVICE_HPP_

#include <rw/kinematics/State.hpp>

/**
 * @brief interface for classes (ODEDevices) that control a set of ode bodies
 * that map to a RWSim dynamic device type.
 */
class ODEDevice {
public:
	/**
	 * @brief destructor
	 */
    virtual ~ODEDevice(){};

    /**
     * @brief resets the ODE device to the state values of the RWSim device.
     * @param state
     */
    virtual void reset(rw::kinematics::State& state) = 0;

    /**
     * @brief the update call is made prior to the simulation step. In this
     * method states of the ODE bodies and joints (forces, velocities, eg)
     * can be updated from the state of the RWSim device.
     * @param dt
     * @param state [out] ODEDevice state values are copied to \b state
     */
    virtual void update(double dt, rw::kinematics::State& state) = 0;

    /**
     * @brief The post update is called after a simulation step has
     * been performed. Here the modified states (force,velocity,position)
     * of the ODE device is written back to the \b state object.
     * @param state
     */
    virtual void postUpdate(rw::kinematics::State& state) = 0;

protected:
    ODEDevice(){};
};

#endif /* ODEDEVICE_HPP_ */
