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

    /** @addtogroup camera */
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

        /**
		 * @brief Set buffer size
		 * @return true
		 *
		 * Need to be restarted afterwards, the camera buffer are by default set to 10
		 **/
        bool setBufferSize(unsigned int bufferSize);

        /**
		 * @brief Get format7 mode
		 * @return format7 mode
		 **/
        CameraFirewire::Format7Mode getFormat7Mode();

        /**
		 * @brief Set format7 mode
		 * @param mode format7 mode to use
		 * @return true
		 *
		 * Need to be reinitialised afterwards
		 **/
        bool setFormat7Mode(Format7Mode mode);

        /**
		 * @brief Set format7 image size
		 * @param width
		 * @param heigth
		 * @return true
		 *
		 * Need to be reinitialised afterwards
		 **/
        bool setFormat7ImageSize(const unsigned int width, const unsigned int heigth);

        /**
		 * @brief Get format7 image size
		 * @param mode format7 mode to use
		 * @param width
		 * @param heigth
		 * @return true
		 *
		 **/
        bool getFormat7ImageSize(const CameraFirewire::Format7Mode mode, unsigned int &width, unsigned int &heigth);

        /**
		 * @brief Get format7 max image size
		 * @param mode format7 mode to use
		 * @param width
		 * @param heigth
		 * @return true if the size could be received from the camera
		 **/
        bool getFormat7ImageMaxSize(const CameraFirewire::Format7Mode mode, unsigned int &width, unsigned int &heigth);

        /**
		 * @brief Set format7 image size to max size
		 * @param mode format7 mode to use
		 * @return true if the size could be received from the camera
		 **/
        bool setFormat7ImageSizeToMax(const CameraFirewire::Format7Mode mode);

        /**
		 * @brief Set format7 image offset position
		 * @param left
		 * @param right
		 * @return true
		 *
		 * Need to be reinitialised afterwards
		 **/
        bool setFormat7ImagePos(const unsigned int left, const unsigned int top);

        /**
		 * @brief Get format7 image offset position
		 * @param left
		 * @param right
		 * @return true if the position could be received from the camera
		 **/
        bool getFormat7ImagePos(unsigned int &left, unsigned int &top);

        /**
		 * @brief Get format7 color coding
		 * @param color
		 * @return true if the color coding could be received from the camera
		 **/
        bool getFormat7ColorCoding(CameraFirewire::ColorCode &color);

        /**
		 * @brief Set format7 packet size to adjust frame rate i format7 mode
		 * @param packetSize
		 * @return true
		 *
		 * Need to be reinitialised afterwards
		 **/
        bool setFormat7PacketSize(const unsigned int packetSize);

        /**
		 * @brief Get format7 packet size
		 * @param packetSize
		 * @return true if the packet size could be received from the camera
		 **/
        bool getFormat7PacketSize(unsigned int &packetSize);

        /**
		 * @brief Get format7 packet size
		 * @param mode the format7 mode that will be used
		 * @param packetSize
		 * @return true if the packet size could be received from the camera
		 **/
        bool getFormat7RecommendedPacketSize(const CameraFirewire::Format7Mode mode, unsigned int &packetSize);

    private:
        //bool _connected;
        CameraFirewire::CapturePolicy _policy;
        bool _isAcquired;
        dc1394camera_t* _dccamera;
        rw::sensor::Image *_image;
        CaptureMode _captureMode;
        ColorCode _colorMode;
        int _frameRate;
        unsigned int _width, _height;
        unsigned int _bufferSize;

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
