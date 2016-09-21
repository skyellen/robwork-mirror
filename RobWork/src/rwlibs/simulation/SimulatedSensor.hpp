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


#ifndef RWLIBS_SIMULATION_SIMULATEDSENSOR_HPP_
#define RWLIBS_SIMULATION_SIMULATEDSENSOR_HPP_

//! @file SimulatedSensor.hpp

#include <rw/sensor/SensorModel.hpp>
#include <rw/common/Ptr.hpp>
#include "Simulator.hpp"

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }

namespace rwlibs {
namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief simulated sensor interface
     */
    class SimulatedSensor: public rw::kinematics::Stateless {
    public:
        //! @brief smart pointer type of this class
        typedef rw::common::Ptr<SimulatedSensor> Ptr;

    protected:
        //! constructor
        SimulatedSensor(rw::sensor::SensorModel::Ptr model):_model(model){}

    public:

        //! @brief destructor
        virtual ~SimulatedSensor();

        /**
         * @brief get name of this simulated sensor
         */
        const std::string& getName() const { return _model->getName(); }

        /**
         * @brief get frame that this sensor is attached to.
         * @return frame
         */
        rw::kinematics::Frame* getFrame() const { return _model->getFrame(); }

        /**
         * @brief steps the the SimulatedSensor with time \b dt and saves any state
         *  changes in \b state.
         * @param info [in] update information related to the time step.
         * @param state [out] changes of the SimulatedSensor is saved in state.
         */
        virtual void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state) = 0;

        /**
         * @brief Resets the state of the SimulatedSensor to that of \b state
         * @param state [in] the state that the sensor is reset too.
         */
        virtual void reset(const rw::kinematics::State& state) = 0;

        /**
         * @brief get the sensor model of this simulated sensor. 
         */
        rw::sensor::SensorModel::Ptr getSensorModel() { return _model; }

        /**
         * get a handle to controlling an instance of the simulated sensor in a specific simulator
         * @param sim [in] the simulator in which the handle is active
         */
        rw::sensor::Sensor::Ptr getSensorHandle(rwlibs::simulation::Simulator::Ptr sim);

    private:
        rw::sensor::SensorModel::Ptr _model;

    };
    //! @}
}
}

#endif /* SIMULATEDSENSOR_HPP_ */
