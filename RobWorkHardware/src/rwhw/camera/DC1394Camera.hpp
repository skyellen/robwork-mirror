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

#ifndef RWHW_DC1394CAMERA_HPP
#define RWHW_DC1394CAMERA_HPP

/**
 * @file DC1394Camera.hpp
 */

#include <rw/sensor/CameraFirewire.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>


//#include <libraw1394/raw1394.h>
//#include <libdc1394/dc1394_control.h>
#include <dc1394/dc1394.h>

#include <vector>

namespace rwhw { namespace camera {

    /** @addtogroup rwhw */
    /* @{ */

    /**
     * @brief The LinuxCamera class implements the Camera interface using
     * the dc1394 linux drivers.
     */
    class DC1394Camera : public rw::sensor::CameraFirewire
    {
    public:
//    protected:
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
            dc1394camera_t* dc1394cam);

//    public:
        /**
         * @brief destructor
         */
        virtual ~DC1394Camera();

        /**
         * @brief get the name of the camera
         * @return return string of camera name
         */
        std::string getCameraName();

        /**
		 * @brief get the vendor of the camera
		 * @return return string of camera vendor
		 */
		std::string getCameraVendor();

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
        virtual CameraFirewire::CaptureMode getCaptureMode();

        /**
         * @brief Get the color mode from the camera
         * @return camera color mode
         */
        CameraFirewire::ColorCode getColorMode();

        /**
         * @copydoc rw::sensor::Camera::setCaptureMode
         */
        virtual bool setCaptureMode(CameraFirewire::CaptureMode mode);

        /**
		 * @copydoc rw::sensor::Camera::setColorMode
		 */
        virtual bool setColorMode(CameraFirewire::ColorCode);

        /**
         * @copydoc rw::sensor::Camera::getCapturePolicy
         */
        CameraFirewire::CapturePolicy getCapturePolicy();

        /**
		 * @copydoc rw::sensor::Camera::setCapturePolicy
		 */
		 bool setCapturePolicy(CameraFirewire::CapturePolicy);

        /**
         * @copydoc rw::sensor::Camera::geFeature
         */
        virtual double getFeature(CameraFirewire::CameraFeature setting);

        /**
         * @copydoc rw::sensor::Camera::setFeature
         */
        virtual bool setFeature(CameraFirewire::CameraFeature setting, double value);

        /**
         * @copydoc rw::sensor::Camera::getWidth
         */
        unsigned int getWidth(){ return _width;};

        /**
         * @copydoc rw::sensor::Camera::getHeight
         */
        unsigned int getHeight(){return _height;};

    private:
        bool _connected;
        CameraFirewire::CapturePolicy _policy;
        bool _isAcquired;
        dc1394camera_t* _dccamera;
        rw::sensor::Image *_image;
        CaptureMode _captureMode;
        ColorCode _colorMode;
        int _frameRate;
        unsigned int _width, _height;

        void acquireOneShot();
        void acquireContinues();
    };

    /* @} */

} } // end namespaces

#endif // end include guard
