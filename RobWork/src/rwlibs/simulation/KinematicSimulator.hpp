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


#ifndef RWLIBS_SIMULATION_KINEMATICSIMULATOR_HPP_
#define RWLIBS_SIMULATION_KINEMATICSIMULATOR_HPP_

//! @file KinematicSimulator.hpp

#include "Simulator.hpp"

namespace rwlibs {
namespace simulation {
	class SimulatedSensor;
	class SimulatedController;

    //! @addtogroup simulation
	// @{
    /**
     * @brief interface of a kinematic simulator
     */
    class KinematicSimulator: public Simulator {
    public:
    	//! smart pointer type of KinematicSimulator
    	typedef rw::common::Ptr<KinematicSimulator> Ptr;

    	/**
         * @brief add a simulated controller to the simulator
         * @param controller [in] the controller to be simulated
         */
        virtual void addController(SimulatedController *controller);

        /**
         * @brief add a simulated sensor to the simulator
         * @param sensor [in] the sensor
         */
        virtual void addSensor(SimulatedSensor *sensor);

        /**
         * @brief remove a sensor from simulation
         * @param sensor [in] sensor that is to be removed
         */
        virtual void removeSensor(SimulatedSensor *sensor);

        /**
         * @brief add a simulator that is to be controlled by this simulator
         * @param sim
         */
        void addSimulator(Simulator *sim);

    	//! @copydoc Simulator::step
    	void step(double dt);

    };
    //! @}
}}

#endif /* KINEMATICSIMULATOR_HPP_ */
