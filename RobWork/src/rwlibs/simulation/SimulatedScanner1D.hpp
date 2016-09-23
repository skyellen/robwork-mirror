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

#ifndef RWLIBS_SIMLATION_SIMULATEDSCANNER25D_HPP
#define RWLIBS_SIMLATION_SIMULATEDSCANNER25D_HPP


//! @file SimulatedScanner1D.hpp

#include <rw/math/Constants.hpp>
#include <rw/sensor/Scan2D.hpp>
#include <rw/sensor/Scanner2D.hpp>
#include "FrameGrabber25D.hpp"
#include "SimulatedSensor.hpp"

namespace rwlibs { namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief A simulated line scanner.
     */
    class SimulatedScanner1D : public rw::sensor::Scanner1D, public virtual SimulatedSensor
    {
    public:
        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
         */
        SimulatedScanner1D(const std::string& name,
                FrameGrabber25D::Ptr framegrabber);

        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param desc [in] description of this scanner
         * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
         */
        SimulatedScanner1D(const std::string& name,
                const std::string& desc,
                FrameGrabber25D::Ptr framegrabber);

        /**
         * @brief destructor
         */
        virtual ~SimulatedScanner1D();

        /**
         * @brief set the framerate in frames per sec.
         * @param rate [in] frames per sec
         */
        void setFrameRate(double rate);

        ///////////// below is inheritet functions form Scanner25D and Sensor

        //! @copydoc Scanner1D::open
        void open();

        //! @copydoc Scanner1D::isOpen
        bool isOpen();

        //! @copydoc Scanner1D::close
        void close();

        //! @copydoc Scanner1D::acquire
        void acquire();

        //! @copydoc Scanner1D::isScanReady
        bool isScanReady();

        //! @copydoc Scanner1D::getRange
        std::pair<double,double> getRange();

        //! @copydoc Scanner1D::getFrameRate
        double getFrameRate();

        //! @copydoc Scanner1D::getImage
        const rw::geometry::PointCloud& getImage();

        /**
         * @brief steps the the sensor with time \b dt and saves any state
         *  changes in \b state.
         * @param dt [in] the timestep.
         * @param state [out] changes of the SimulatedSensor is saved in state.
         */
        void update(double dt, rw::kinematics::State& state);

        //! @copydoc SimulatedSensor::reset
        void reset(const rw::kinematics::State& state);

        //! @copydoc SimulatedSensor::getSensor
        rw::sensor::Sensor* getSensor();


        //! @copydoc SimulatedSensor::getData
        virtual const rw::geometry::PointCloud& getData() {
            return _scan;
        }

        //! @copydoc SimulatedSensor::getResolution
        virtual double getResolution() {
            return _framegrabber->getFieldOfViewY()*rw::math::Deg2Rad/_scan.getWidth();
        }


    private:
        FrameGrabber25D::Ptr _framegrabber;
        double _frameRate, _dtsum;
        bool _isAcquired,_isOpenned;
        rw::geometry::PointCloud _scan;
    };

    /**
     * @brief Definition of pointer to SimulatedScanner2D
     */
    typedef rw::common::Ptr<SimulatedScanner2D> SimulatedScanner2DPtr;

    //! @}
}
}


#endif /* SIMULATEDSCANNER25D_HPP_ */
