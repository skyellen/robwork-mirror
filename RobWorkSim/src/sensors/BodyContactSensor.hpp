/*
 * BodyForceSensor.hpp
 *
 *  Created on: 10-09-2009
 *      Author: jimali
 */

#ifndef BODYCONTACTSENSOR_HPP_
#define BODYCONTACTSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/Contact3D.hpp>
#include <rw/sensor/Sensor.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/multi_array.hpp>

/**
 * @brief This sensor attaches to a body and records all forces and the corresponding
 * positions where the forces act.
 *
 */
class BodyContactSensor: public SimulatedTactileSensor, public rw::sensor::Sensor {
public:
    /**
     * @brief constructor
     * @param name [in] the sensor name
     * @param frame [in] the frame of this sensor.
     * @return
     */
    BodyContactSensor(const std::string& name, rw::kinematics::Frame* frame);

    /**
     * @brief Destructor
     * @return
     */
    virtual ~BodyContactSensor();

    //// Interface inherited from SimulatedSensor
    void update(double dt, rw::kinematics::State& state);

    void reset(const rw::kinematics::State& state);

    rw::sensor::Sensor* getSensor();


    //// Inherited from SimulatedTactileSensor
    void addForceW(const rw::math::Vector3D<>& point,
                   const rw::math::Vector3D<>& force,
                   const rw::math::Vector3D<>& cnormal,
                   dynamics::Body *body = NULL);

    virtual void addForce(const rw::math::Vector3D<>& point,
                   const rw::math::Vector3D<>& force,
                   const rw::math::Vector3D<>& cnormal,
                   dynamics::Body *body = NULL);

    // now for the functions belonging to this class
    /**
     * @brief return all contacts registered in the last timestep
     * @return
     */
    const std::vector<Contact3D>& getContacts(){
        return _contacts;
    }


private:
    // hmm,
    std::vector<Contact3D> _contactsTmp,_contacts;
    rw::math::Transform3D<> _wTf, _fTw;
};

typedef rw::common::Ptr<BodyContactSensor> BodyContactSensorPtr;

#endif /* BODYFORCESENSOR_HPP_ */
