/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_SENSOR_FTSENSORMODEL_HPP
#define RW_SENSOR_FTSENSORMODEL_HPP

#include "Sensor.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/multi_array.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

#include "SensorModel.hpp"

namespace rw {
namespace sensor {

/**
 * @brief N-axis Force Torque sensor model
 */
class FTSensorModel : public SensorModel {
public:
	//! smart pointer type
	typedef rw::common::Ptr<FTSensorModel> Ptr;

    /**
     * Constructor
     * @param name [in] name of FT sensor
     * @param frame [in] the frame to which this sensor is attached
     * @param desc [in] optional description of sensor
     */
    FTSensorModel(const std::string& name, rw::kinematics::Frame* frame, const std::string& desc="");

    /**
     * @brief destructor
     * @return
     */
    virtual ~FTSensorModel();

    /**
     * @brief get maximum wrench (force and torque) characteristics
     * @return
     */
    rw::math::Wrench6D<> getMaxWrench() const;

    /**
     * @brief gets the maximum force in Newton that this sensor can measure on any of its
     * axis.
     * @return max force in Newton.
     */
    rw::math::Vector3D<> getMaxForce() const;

    /**
     * @brief gets the maximum torque in Newton Meter (N m)that this sensor can measure on any of its
     * axis.
     * @return max torque in Newton Meter(N m).
     */
    rw::math::Vector3D<> getMaxTorque() const;

    /**
     * @brief set the maximum wrench of this FTSensor
     * @param max [in] maximum allowed wrench
     */
    void setMaxWrench(const rw::math::Wrench6D<>& max);

    /**
     * @brief gets the force in N that is acting on the origin. The
     * force is described in relation to the origin.
     * @return force acting on origin.
     */
    rw::math::Wrench6D<> getWrench(const rw::kinematics::State& state) const;

    //! set the wrench that is acting on the origin of this FTsensor
    void setWrench(const rw::math::Wrench6D<>& wrench, const rw::kinematics::State& state);

    /**
     * @brief gets the force in N that is acting on the origin. The
     * force is described in relation to the origin.
     * @return force acting on origin.
     */
    rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const;
    //! set the force that is acting on the origin of this FTsensor
    void setForce(const rw::math::Vector3D<>& force, const rw::kinematics::State& state);

    /**
     * @brief gets the torgue in Nm that is acting on the origin. The
     * torque is described in relation to the origin.
     * @return torque acting on origin.
     */
    rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const;
    //! set the torque that is acting on the origin of this FTsensor
    void setTorque(const rw::math::Vector3D<>& force, const rw::kinematics::State& state);

    /**
     * @brief the transform from the sensor frame to the point of origin.
     * @return transform from sensor frame to point of origin.
     */
    rw::math::Transform3D<> getTransform() const;

    /**
     * @brief set the transform between frame and origin. The origin of the
     * sensor is the frame where sensor data is described.
     * @param t3d [in] transformation from frame to origin
     */
    void setTransform(const rw::math::Transform3D<>& t3d);

private:
    rw::math::Wrench6D<> _maxWrench;
    rw::math::Transform3D<> _offset;

    rw::kinematics::StatelessData<rw::math::Wrench6D<> > _sdata;
};

}
}

#endif /*RW_SENSOR_TACTILEARRAY_HPP*/
