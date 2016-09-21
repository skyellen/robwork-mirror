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

#include <string>

namespace rw { namespace sensor { class CameraModel; } }

namespace rwlibs { namespace simulation {
	class FrameGrabber;

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
		//! @brief Smart pointer type for a SimulatedCamera.
        typedef rw::common::Ptr<SimulatedCamera> Ptr;

        /**
         * @brief creates a simulated pinhole camera,
         * @param name [in] name of sensor
         * @param fov [in] field of view for the camera.
         * @param frame [in] frame to which the camera is attached
         * @param frameGrabber [in] the frameGrabber from which this Camera should grab images
         */
        SimulatedCamera(const std::string& name, double fov, rw::kinematics::Frame* frame, rw::common::Ptr<FrameGrabber> frameGrabber);

        /**
         * @brief constructor
         * @param model [in] the model and info of the camera
         * @param frameGrabber [in] the frameGrabber from which this Camera should grab
         * images.
         */
        SimulatedCamera(rw::common::Ptr<rw::sensor::CameraModel> model, rw::common::Ptr<FrameGrabber> frameGrabber);

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
        virtual unsigned int getWidth() const;

        /**
         * @copydoc rw::sensor::Camera::getHeight
         */
        virtual unsigned int getHeight() const;

        /**
         * @copydoc SimulatedSensor::update
         */
        void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

        /**
         * @copydoc SimulatedSensor::reset
         */
        void reset(const rw::kinematics::State& state){}

        /**
         * @copydoc SimulatedSensor::getSensor
         */
        rw::sensor::Sensor::Ptr getSensor(){ return _csensor; };

        /**
         * @brief Get the camera sensor.
         * @return the sensor.
         */
        rw::sensor::Camera::Ptr getCameraSensor(){ return _csensor; };

    private:
        void acquire(char *imgData);

    private:
        double _frameRate;
        double _dtSum;
        rw::common::Ptr<FrameGrabber> _frameGrabber;
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
