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

#define REG_CAMERA_AVT_SOFT_RESET	0x510U

#include <rw/sensor/CameraFirewire.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>


//#include <libraw1394/raw1394.h>
//#include <libdc1394/dc1394_control.h>
#include <dc1394/dc1394.h>
#include <dc1394/format7.h>

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
         * @copydoc rw::sensor::Camera::setFeature
         */
        virtual bool setFeature(CameraFirewire::CameraFeature setting, std::vector<double> values);

        /**
         * @copydoc rw::sensor::Camera::getWidth
         */
        unsigned int getWidth(){ return _width;};

        /**
         * @copydoc rw::sensor::Camera::getHeight
         */
        unsigned int getHeight(){return _height;};


        CameraFirewire::Format7Mode getFormat7Mode();
        bool setFormat7Mode(Format7Mode mode);

        bool setFormat7ImageSize(const unsigned int width, const unsigned int heigth);
        bool getFormat7ImageSize(const CameraFirewire::Format7Mode mode, unsigned int &width, unsigned int &heigth);
        bool getFormat7ImageMaxSize(const CameraFirewire::Format7Mode mode, unsigned int &width, unsigned int &heigth);
        bool setFormat7ImageSizeToMax(const CameraFirewire::Format7Mode mode);

        bool setFormat7ImagePos(const unsigned int left, const unsigned int top);
        bool getFormat7ImagePos(unsigned int &left, unsigned int &top);

        bool getFormat7ColorCoding(CameraFirewire::ColorCode &color);

        bool setFormat7PacketSize(const unsigned int packetSize);
        bool getFormat7PacketSize(unsigned int &packetSize);
        bool getFormat7RecommendedPacketSize(const CameraFirewire::Format7Mode mode, unsigned int &packetSize);

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

        //Format7
        Format7Mode _f7Mode;
        unsigned int _f7width, _f7height;
        unsigned int _f7PosLeft, _f7PosTop;
        unsigned int _f7PacketSize;


        void acquireOneShot();
        void acquireContinues();

        dc1394video_mode_t modeConverter(CameraFirewire::Format7Mode color);
        CameraFirewire::Format7Mode modeConverter(dc1394video_mode_t color);

        dc1394color_coding_t ColorCodeConverter(CameraFirewire::ColorCode color);
        CameraFirewire::ColorCode ColorCodeConverter(dc1394color_coding_t color);

        dc1394feature_t settingsConverter(CameraFirewire::CameraFeature setting);
        CameraFirewire::CameraFeature settingsConverter(dc1394feature_t setting);
    };

    /* @} */

} } // end namespaces

#endif // end include guard
