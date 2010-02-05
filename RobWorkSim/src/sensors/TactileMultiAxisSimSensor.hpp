/*
 * TactileMultiAxisSimSensor.hpp
 *
 *  Created on: 25-08-2008
 *      Author: jimali
 */

#ifndef TACTILEMULTIAXISSIMSENSOR_HPP_
#define TACTILEMULTIAXISSIMSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/TactileMultiAxisSensor.hpp>


/**
 * @brief A sensor that measures force and torque around some reference frame
 */
class TactileMultiAxisSimSensor: public rw::sensor::TactileMultiAxisSensor, public SimulatedTactileSensor {
public:

    TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body);

    virtual ~TactileMultiAxisSimSensor(){};

    //// Interface inherited from SimulatedSensor
    void update(double dt, rw::kinematics::State& state);

    void reset(const rw::kinematics::State& state);

    //// Interface inherited from SimulatedTactileSensor
    void addForceW(const rw::math::Vector3D<>& point,
                   const rw::math::Vector3D<>& force,
                   const rw::math::Vector3D<>& cnormal,
                   dynamics::Body *body = NULL);

    void addForce(const rw::math::Vector3D<>& point,
                  const rw::math::Vector3D<>& force,
                  const rw::math::Vector3D<>& cnormal,
                  dynamics::Body *body=NULL);


    /**
     * @brief the transform from the sensor frame to the point of origin.
     * @return transform from sensor frame to point of origin.
     */
    rw::math::Transform3D<> getTransform();

    /**
     *
     * @return
     */
    rw::math::Vector3D<> getForce();

    /**
     * @brief
     * @return
     */
    rw::math::Vector3D<> getTorque();

};


#endif /* TACTILEMULTIAXISSENSOR_HPP_ */
