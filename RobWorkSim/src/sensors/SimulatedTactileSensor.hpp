/*
 * SimulatedTactileSensor.hpp
 *
 *  Created on: 20-10-2008
 *      Author: jimali
 */

#ifndef SIMULATEDTACTILESENSOR_HPP_
#define SIMULATEDTACTILESENSOR_HPP_

#include <sandbox/simulation/SimulatedSensor.hpp>
#include <dynamics/Body.hpp>

/**
 * @brief interface for simulated tactile sensors
 */
class SimulatedTactileSensor: public rwlibs::simulation::SimulatedSensor {
public:
    /**
     * @brief add a force to a point on the sensor geometry. The force is described
     * relative to the world frame.
     * @param point [in] the point where the force is acting.
     * @param force [in] the direction in which the force is acting
     * @param cnormal [in] the contact normal where the origin is on the
     * contacting body and the direction is toward the sensor
     * @param body [in] the body that caused the contact force. If no body
     * caused the force on the sensor (could be user input) then the body is NULL
     *
     */
    virtual void addForceW(const rw::math::Vector3D<>& point,
                   const rw::math::Vector3D<>& force,
                   const rw::math::Vector3D<>& cnormal,
                   dynamics::Body *body = NULL) = 0;

    /**
     * @brief add a force to a point on the sensor geometry. The force is described
     * relative to the sensor frame.
     * @param point [in] the point where the force is acting.
     * @param force [in] the direction in which the force is acting
     * @param cnormal [in] the contact normal where the origin is on the
     * contacting body and the direction is toward the sensor
     * @param body [in] the body that caused the contact force. If no body
     * caused the force on the sensor (could be user input) then the body is NULL
     */
    virtual void addForce(const rw::math::Vector3D<>& point,
                  const rw::math::Vector3D<>& force,
                  const rw::math::Vector3D<>& cnormal,
                  dynamics::Body *body=NULL) = 0;


};

#endif /* SIMULATEDTACTILESENSOR_HPP_ */
