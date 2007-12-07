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

#ifndef rwlibs_sensors_camera_DC1394Camera_HPP
#define rwlibs_sensors_camera_DC1394Camera_HPP

/**
 * @file DC1394Camera.hpp
 */

#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>

#include <vector>

namespace rwlibs { namespace sensors {

    /** @addtogroup sensors */
    /* @{ */

    /**
     * @brief The LinuxCamera class implements the Camera interface using
     * the dc1394 linux drivers.
     */
    class DC1394Camera : public rw::sensor::Camera
    {
    protected:
        /**
         * @brief constructs ad DC1394 camera
         *
         * @param frame [in] frame in which the camera is located
         *
         * @param handle [in] handle to firewire device
         *
         * @param dc1394cam [in] handle to camera
         */
        DC1394Camera(
            rw::kinematics::Frame* frame,
            raw1394handle_t handle, dc1394_cameracapture dc1394cam);

    public:
        /**
         * @brief destructor
         */
        virtual ~DC1394Camera();

        /**
         * @brief return handles (Camera objects) to all connected
         * firewire cameras.
         * @return a list of available cameras
         */
        static const std::vector<DC1394Camera*> getCameraHandles();

        /**
         * @copydoc rw::sensor::Camera::initialize
         */
        bool initialize();

        /**
         * @copydoc rw::sensor::Camera::stop
         */
        void stop();

        /**
         * @copydoc rw::sensor::Camera::start
         */
        bool start();

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
         * @copydoc rw::sensor::Camera::getCaptureMode
         */
        virtual CaptureMode getCaptureMode();

        /**
         * @copydoc rw::sensor::Camera::setCaptureMode
         */
        virtual bool setCaptureMode(CaptureMode mode);

        /**
         * @copydoc rw::sensor::Camera::getDimension
         */
        std::pair<unsigned int,unsigned int> getDimension();

        /**
         * @copydoc rw::sensor::Camera::getCapturePolicy
         */
        CapturePolicy getCapturePolicy();

    private:
        bool _connected;
        CapturePolicy _policy;
        bool _isAcquired;
        raw1394handle_t _handle;
        dc1394_cameracapture _dccamera;
        rw::sensor::Image *_image;
        int _captureMode;
        int _frameRate;
    };

    /* @} */

}} // end namespaces

#endif // end include guard
