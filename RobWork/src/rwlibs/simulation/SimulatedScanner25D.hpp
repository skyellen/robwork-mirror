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

#ifndef RWLIBS_SIMULATION_SIMULATEDSCANNER25D_HPP_
#define RWLIBS_SIMULATION_SIMULATEDSCANNER25D_HPP_

//! @file SimulatedScanner25D.hpp

#include <rw/common/Ptr.hpp>
#include <rw/sensor/Scanner25D.hpp>
#include "FrameGrabber25D.hpp"
#include "SimulatedSensor.hpp"

#include <rw/sensor/Scanner25DModel.hpp>

namespace rwlibs { namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief a simulated range scanner for 2.5D images, that is basically
     * pointclouds without color information.
     */
    class SimulatedScanner25D : public SimulatedSensor
    {
    public:
        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
         */
    	SimulatedScanner25D(const std::string& name,
    	                    rw::kinematics::Frame *frame,
    	                    FrameGrabber25D::Ptr framegrabber);

        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param desc [in] description of this scanner
         * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
         */
    	SimulatedScanner25D(const std::string& name,
                            const std::string& desc,
                            rw::kinematics::Frame *frame,
                            FrameGrabber25D::Ptr framegrabber);

    	/**
    	 * @brief destructor
    	 */
    	virtual ~SimulatedScanner25D();

    	/**
    	 * @brief set the framerate in frames per sec.
    	 * @param rate [in] frames per sec
    	 */
    	void setFrameRate(double rate);

    	///////////// below is inheritet functions form Scanner25D and Sensor

    	//! @copydoc Scanner25D::open
        void open();

        //! @copydoc Scanner25D::isOpen
        bool isOpen();

        //! @copydoc Scanner25D::close
        void close();

        //! @copydoc Scanner25D::acquire
        void acquire();

        //! @copydoc Scanner25D::isScanReady
        bool isScanReady();

        //! @copydoc Scanner25D::getRange
        std::pair<double,double> getRange();

        //! @copydoc Scanner25D::getFrameRate
        double getFrameRate();

        //! @copydoc Scanner25D::getImage
    	const rw::geometry::PointCloud& getScan();

    	//! @copydoc SimulatedSensor::update
        void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::reset
    	void reset(const rw::kinematics::State& state);

    	//! @copydoc rw::simulation::SimulatedSensor::getSensorHandle
        rw::sensor::Sensor::Ptr getSensorHandle(rwlibs::simulation::Simulator::Ptr instance);

    	//! get instance of scanner
        rw::sensor::Scanner25D::Ptr getScanner25DSensor(rwlibs::simulation::Simulator::Ptr instance);

    private:
        FrameGrabber25D::Ptr _framegrabber;
        double _frameRate, _dtsum;
        bool _isAcquired,_isOpenned;
    };

    //! @}
}
}

#endif /* SIMULATEDSCANNER25D_HPP_ */
