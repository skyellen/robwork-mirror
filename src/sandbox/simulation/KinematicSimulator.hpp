/*
 * KinematicSimulator.hpp
 *
 *  Created on: 20-10-2008
 *      Author: jimali
 */

#ifndef KINEMATICSIMULATOR_HPP_
#define KINEMATICSIMULATOR_HPP_

namespace rwlibs {
namespace simulation {

class KinematicSimulator {
private:

    void step(double dt, rw::kinematics::State &state);

    void addController(Controller *controller);

    void addSensor(SimulatedSensor *sensor);
    void removeSensor(SimulatedSensor *sensor);

};

}}

#endif /* KINEMATICSIMULATOR_HPP_ */
