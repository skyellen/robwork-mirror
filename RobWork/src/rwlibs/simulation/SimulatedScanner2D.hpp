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

#ifndef RWLIBS_SIMULATION_SIMULATEDSCANNER2D_HPP
#define RWLIBS_SIMULATION_SIMULATEDSCANNER2D_HPP

//! @file SimulatedScanner2D.hpp

#include <rw/math/Constants.hpp>
#include <rw/sensor/Scan2D.hpp>
#include <rw/sensor/Scanner2D.hpp>
#include "FrameGrabber25D.hpp"
#include "SimulatedSensor.hpp"

namespace rwlibs { namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief
     */
    class SimulatedScanner2D : public rw::sensor::Scanner2D, public virtual SimulatedSensor
    {
    public:
        //! @brief smart pointer of this class
        typedef rw::common::Ptr<SimulatedScanner2D> Ptr;

        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
         */
        SimulatedScanner2D(const std::string& name,
                FrameGrabber25DPtr framegrabber);

        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param desc [in] description of this scanner
         * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
         */
        SimulatedScanner2D(const std::string& name,
                const std::string& desc,
                FrameGrabber25DPtr framegrabber);

        /**
         * @brief destructor
         */
        virtual ~SimulatedScanner2D();

        /**
         * @brief set the framerate in frames per sec.
         * @param rate [in] frames per sec
         */
        void setFrameRate(double rate);

        ///////////// below is inheritet functions form Scanner25D and Sensor

        //! @copydoc Scanner2D::open
        void open();

        //! @copydoc Scanner2D::isOpen
        bool isOpen();

        //! @copydoc Scanner2D::close
        void close();

        //! @copydoc Scanner2D::acquire
        void acquire();

        //! @copydoc Scanner2D::isScanReady
        bool isScanReady();

        //! @copydoc Scanner2D::getRange
        std::pair<double,double> getRange();

        //! @copydoc Scanner2D::getFrameRate
        double getFrameRate();

        //! @copydoc SimulatedSensor::getScan
        const rw::sensor::Image25D& getScan() const;

        //! @copydoc SimulatedSensor::update
        void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

        //! @copydoc SimulatedSensor::reset
        void reset(const rw::kinematics::State& state);

        //! @copydoc SimulatedSensor::getSensor
        rw::sensor::Sensor* getSensor();

        //! @copydoc SimulatedSensor::getAngularRange
        virtual double getAngularRange() {
            return _framegrabber->getFieldOfViewY();
        }

        //! @copydoc SimulatedSensor::getWidth
        virtual size_t getMeasurementCount() const {
            return _framegrabber->getWidth()*_framegrabber->getHeight();
        }

        rw::kinematics::Frame * getSensorFrame(){ return getFrame(); }
    private:
        FrameGrabber25DPtr _framegrabber;
        double _frameRate, _dtsum;
        bool _isAcquired,_isOpenned;
    };


    //! @}
}
}

#endif /* RWLIBS_SIMULATION_SIMULATEDSCANNER2D_HPP */
