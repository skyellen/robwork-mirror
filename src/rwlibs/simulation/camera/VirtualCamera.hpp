/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rwlibs_simulation_camera_VirtualCamera_HPP
#define rwlibs_simulation_camera_VirtualCamera_HPP

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
        void acquire(const rw::kinematics::State& state);

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
