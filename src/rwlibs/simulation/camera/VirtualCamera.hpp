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


#ifndef RWLIBS_SIMULATION_CAMERA_VIRTUALCAMERA_HPP
#define RWLIBS_SIMULATION_CAMERA_VIRTUALCAMERA_HPP

/**
   @file VirtualCamera.hpp
*/

#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Image.hpp>

#include "FrameGrabber.hpp"

#include <string>
#include <cmath>

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
       @brief The VirtualCamera class makes it posible to use virtual camera
       sensors by using different framegrapper implementations.

       The VirtualCamera implements the camera interface though the setting of
       framerate has no meaning to the virtual camera since no timing is done in
       this implementation.
    */
    class VirtualCamera : public rw::sensor::Camera
    {
    public:
        /**
         * @brief constructor
         * @param name [in] name and model info of camera
         * @param frameGrabber [in] the frameGrabber from which this Camera should grab
         * @param frame [in] frame associated with the camera
         * images.
         */
        VirtualCamera(
            const std::string& name,
            FrameGrabber &frameGrabber,
            rw::kinematics::Frame *frame);

        /**
         * @brief destructor
         */
        virtual ~VirtualCamera();

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
         * @copydoc rw::sensor::Camera::getDimension
         */
        std::pair<unsigned int,unsigned int> getDimension();

        /**
         * @copydoc rw::sensor::Camera::getCapturePolicy
         */
        CapturePolicy getCapturePolicy();

        /**
         * @copydoc rw::sensor::Camera::getCaptureMode
         */
        CaptureMode getCaptureMode();

        /**
         * @copydoc rw::sensor::Camera::setCaptureMode
         */
        bool setCaptureMode(CaptureMode mode);

        void update(double dt, const rw::kinematics::State& state);

    private:
        void acquire(char *imgData);

    private:
        std::string _name;
        CapturePolicy _policy;
        double _frameRate;
        CaptureMode _mode;
        FrameGrabber *_frameGrabber;
        ErrorCode _error;
        bool _isAcquired;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
