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


#ifndef RW_SENSOR_CAMERAFIREWIRE_HPP
#define RW_SENSOR_CAMERAFIREWIRE_HPP

/**
 * @file CameraFirewire.hpp
 */

#include "Camera.hpp"

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief The Camera class defines a generel interface to a camera.
     * A great deal of the interface resembles the DCAM standard since
     * DCAM allready defines a very wide interface.
     *
     * typical usage:
     * Camera c;
     * // setup camera features modes and so on
     * c.initialize();
     * c.start();
     * // acquire images
     * c.stop();
     *
     */
    class CameraFirewire : public Camera
    {
    public:
        //! @brief Optional features of a camera
        enum CameraFeature {
            SHUTTER,ZOOM,GAIN,FOCUS,IRIS,HUE,
            WHITEBALANCE,SHARPNESS,SATURATION,
            GAMMA,BRIGHTNESS,AUTOEXPOSURE
        };

        //! @brief Optional colormodes available when capturing
        enum ColorCode {
            MONO8,YUV411,YUV422,YUV444,RGB8,
            MONO16,RGB16,MONO16S,RGB16S,RAW8,
            RAW16,RGB24
        };

        /**
         * @brief defines how images are captured.
         * When the SINGLE_SHOT CapturePolicy is used the user has to trigger
         * the camera through acquire. When CONTINUES is used images are captured
         * according to the given frameRate and getImage is more efficient to use.
         * The image returned is allways the newest captured image.
         * When using CONTINUES_BUFFERED images are continuesly captured and put in
         * a buffer.
         */
        enum CapturePolicy {
            SINGLE_SHOT,CONTINUES,CONTINUES_BUFFERED
        };

        //! @brief The resolution of the camera capture
        enum CaptureMode {
            M160x120, M320x240, M640x480, M800x600,
            M1024x768, M1280x960, M1600x1200, MFORMAT7
        };

        //! @brief The resolution of the camera capture
        enum Format7Mode {
            F7MODE0, F7MODE1, F7MODE2, F7MODE3, F7MODE4, F7MODE5, F7MODE6, F7MODE7
        };

        //! @brief Modes of the camera, inspired by the DCAM standard modes
        enum TriggerMode {
            MODE0,MODE1,MODE2,MODE3,
            MODE4,MODE5,MODE14,MODE15
        };

        //! @brief error codes for a camera
        enum ErrorCode {
            SUCCES,FAILURE,NOT_INITIALIZED,NOT_STARTED,
            UNSUPPORTED_CAPTURE_MODE,
            UNSUPPORTED_FEATURE
        };

    protected:
        /**
         * @brief constructor
         * @param frame [in] Frame in which to place the sensor
         * @param name [in] name of sensor
         * @param modelInfo [in] info string
         */
        CameraFirewire(
            const std::string& name,
            const std::string& modelInfo);

    public:
        /**
         * @brief destructor
         */
        virtual ~CameraFirewire();

        /**
         * @brief returns the CaptureMode of this camera
         * @return the camera capturemode
         */
        virtual CaptureMode getCaptureMode() = 0;

        /**
         * @brief sets the CaptureMode of this camera.
         * @param mode [in] the wanted capture mode
         * @return true if CaptureMode was set successfully, false otherwise
         */
        virtual bool setCaptureMode(CaptureMode mode) = 0;


        /**
         * @brief returns the CaptureMode of this camera
         * @return the camera capturemode
         */
        virtual ColorCode getColorMode() = 0;

        /**
         * @brief sets the CaptureMode of this camera.
         * @param mode [in] the wanted capture mode
         * @return true if CaptureMode was set successfully, false otherwise
         */
        virtual bool setColorMode(ColorCode mode) = 0;

        /**
         * @brief returns the errorcode of the latest error. If no error has occured
         * then SUCCES is returned.
         * @return the error code
         */
        virtual ErrorCode getError(){return SUCCES;};

        /**
         * @brief tests whether this camera is in an error state.
         * @return true if camera is in error state, false otherwise
         */
        virtual bool isError(){return false;};

        /**
         * @brief returns the capture policy of this camera.
         * @return capture policy of the camera
         */
        virtual CapturePolicy getCapturePolicy() = 0;

        /**
         * @brief sets the capture policy of this camera
         * @param policy [in] the capture policy
         * @return true if capture policy was set succesfully, false otherwise
         */
        virtual bool setCapturePolicy(CapturePolicy policy){return false;};

        /**
         * @brief returns whether the specified camera option is supported
         * by the camera.
         * @param option [in] the specific CameraOption
         * @return true if the option is available, false otherwise.
         */
        virtual bool isFeatureAvailable(CameraFeature option){return false;};

        /**
         * @brief returns the value of the specified camera setting. If the
         * camera is not initialized or the setting is unsupported -1 is returned.
         * @param setting [in] the CameraFeature
         * @return value of the setting if setting is supported and camera is
         * initilized, else -1 is returned.
         */
        virtual double getFeature(CameraFeature setting){return -1;};

        /**
         * @brief sets the value of the specified camera setting. If the
         * camera is not initialized or the setting is unsupported false is returned.
         * @param setting [in] the CameraFeature
         * @param value [in] the value of the feature
         * @return true if the setting was succesfully changed, false otherwise.
         */
        virtual bool setFeature(CameraFeature setting, double value){return false;};

    protected:

    private:
        CameraFirewire(const CameraFirewire&);
        CameraFirewire& operator=(const CameraFirewire&);
    };

    /* @} */

}} // end namespaces

#endif // end include guard
