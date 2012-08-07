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


#ifndef RWLIBS_SIMULATION_SIMULATEDCAMERA_HPP
#define RWLIBS_SIMULATION_SIMULATEDCAMERA_HPP

/**
   @file SimulatedCamera.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Image.hpp>

#include <rwlibs/simulation/SimulatedSensor.hpp>

#include "FrameGrabber.hpp"

#include <string>
#include <cmath>

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
       @brief The SimulatedCamera class makes it posible to use virtual camera
       sensors by using different framegrapper implementations.

       The SimulatedCamera implements the camera interface though the setting of
       framerate has no meaning to the virtual camera since no timing is done in
       this implementation.
    */
    class SimulatedCamera : public SimulatedSensor
    {
    public:
        typedef rw::common::Ptr<SimulatedCamera> Ptr;

        /**
         * @brief constructor
         * @param name [in] name and model info of camera
         * @param frameGrabber [in] the frameGrabber from which this Camera should grab
         * @param frame [in] frame associated with the camera
         * images.
         */
        SimulatedCamera(const std::string& name, rw::kinematics::Frame *frame, FrameGrabber::Ptr frameGrabber);

        /**
         * @brief destructor
         */
        virtual ~SimulatedCamera();

        /**
         * @copydoc rw::sensor::Camera::initialize
         */
        bool initialize();

        /**
         * @copydoc rw::sensor::Camera::start
         */
        bool start();

        /**
         * @copydoc rw::sensor::Camera::stop
         */
        void stop();

        /**
         * @copydoc rw::sensor::Camera::acquire
         */
        void acquire();

        /**
         * @copydoc rw::sensor::Camera::isImageReady
         */
        bool isImageReady();

        /**
         * @copydoc rw::sensor::Camera::getImage
         */
        const rw::sensor::Image* getImage();

        /**
         * @copydoc rw::sensor::Camera::getFrameRate
         */
        double getFrameRate();

        /**
         * @copydoc rw::sensor::Camera::setFrameRate
         */
        void setFrameRate(double framerate);

        /**
         * @copydoc rw::sensor::Camera::getWidth
         */
        virtual unsigned int getWidth(){return _frameGrabber->getWidth();};

        /**
         * @copydoc rw::sensor::Camera::getHeight
         */
        virtual unsigned int getHeight(){return _frameGrabber->getHeight();};

        /**
         * @copydoc SimulatedSensor::update
         */
        void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

        /**
         * @copydoc SimulatedSensor::reset
         */
        void reset(const rw::kinematics::State& state){};

        /**
         * @copydoc SimulatedSensor::getSensor
         */
        rw::sensor::Sensor::Ptr getSensor(){ return _csensor; };
        rw::sensor::Camera::Ptr getCameraSensor(){ return _csensor; };

    private:
        void acquire(char *imgData);

    private:
        std::string _name;
        double _frameRate;
        double _dtSum;
        FrameGrabber::Ptr _frameGrabber;
        bool _isAcquired;
        bool _started;
        bool _initialized;

        rw::sensor::Camera::Ptr _csensor;
    };

    /**
     * @brief Smart pointer to simulated camera
     */
    typedef rw::common::Ptr<SimulatedCamera> SimulatedCameraPtr;


    /* @} */
}} // end namespaces

#endif // end include guard
