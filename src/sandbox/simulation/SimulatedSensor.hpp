/*
 * SimulatedSensor.hpp
 *
 *  Created on: 20-10-2008
 *      Author: jimali
 */

#ifndef SIMULATEDSENSOR_HPP_
#define SIMULATEDSENSOR_HPP_

#include <rw/kinematics/State.hpp>

namespace rwlibs {
namespace simulation {


class SimulatedSensor {

public:

    /**
     * @brief Updates the state of the SimulatedSensor and saves any state
     *  changes in \b state.
     * @param dt
     * @param state
     */
    virtual void update(double dt, rw::kinematics::State& state) = 0;

};

}}

#endif /* SIMULATEDSENSOR_HPP_ */
