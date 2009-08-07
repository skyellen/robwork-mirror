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


#ifndef RW_SENSOR_CAMERA_HPP
#define RW_SENSOR_CAMERA_HPP

/**
 * @file Camera.hpp
 */

#include "Image.hpp"
#include "Sensor.hpp"
#include "CameraListener.hpp"

#include <rw/kinematics/State.hpp>

#include <vector>
#include <string>

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
    class Camera : public Sensor
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
            M1024x768, M1280x960, M1600x1200
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
        Camera(
            kinematics::Frame* frame,
            const std::string& name,
            const std::string& modelInfo);

        /**
         * @brief sets the camera model information
         * @param info [in] information of the robot
         */
        void setModelInfo(const std::string info) { _modelInfo = info; }

    public:
        /**
         * @brief destructor
         */
        virtual ~Camera();

        /**
         * @brief returns the camera model information (version, type, size, etc.)
         * @return camera model information
         */
        virtual std::string getModelInfo() const { return _modelInfo;};

        /**
         * @brief initializes the camera to the current settings
         * (CaptureMode,ColorMode,etc.)
         * @return true if initialization is succesfully, false otherwise.
         */
        virtual bool initialize() = 0;

        /**
         * @brief returns whether this camera is initialized or not.
         * @return true if intialized, false otherwise
         */
        bool isInitialized() const { return _initialized;};

        /**
         * @brief starts this camera, if the camera has not been
         * initialized the initialize function will be called.
         * @return true if camera was successfully started, false
         * otherwise
         */
        virtual bool start() = 0;

        /**
         * @brief returns whether this camera is started or not.
         * @return true if started, false otherwise
         */
        bool isStarted() const { return _started;};

        /**
         * @brief stops this camera. When the camera is stopped it can be
         * reinitialized using initialize()
         */
        virtual void stop() = 0;

        /**
         * @brief aquires an image from the camera. This method is not blocking.
         * Use  isImageReady to poll for completion of acquire.
         */
        virtual void acquire() = 0;

        /**
         * @brief tests whether a image has been acquired
         * @return true if an image has been acquired, false otherwise.
         */
        virtual bool isImageReady() = 0;

        /**
         * @brief returns the last image acquired from the camera. This method
         * is not blocking, if no image has been acquired yet an empty image
         * is returned. The image returned can for some specific drivers be read
         * only.
         * @return last image captured from camera.
         */
        virtual const Image* getImage() = 0;

        /**
         * @brief returns the framerate that this camera is setup with
         * @return the framerate in frames per second
         */
        virtual double getFrameRate() = 0;

        /**
         * @brief sets the framerate of this camera. If the framerate is not
         * supported the closest supported framerate is choosen.
         * @param framerate [in] the framerate
         */
        virtual void setFrameRate(double framerate) = 0;

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
         * @brief get width and height of the captured images
         * @return width as first element and height as second element
         */
        virtual std::pair<unsigned int,unsigned int> getDimension() = 0;

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
         * @brief adds a CameraListener to this camera
         * @param listener [in] the CameraListener that is to be added
         * @return true if listener was added succesfully, false otherwise
         */
        virtual bool addListener(CameraListener& listener);

        /**
         * @brief removes a CameraListener from this cameras listener list.
         * @param listener [in] the listener that is to be removed
         * @return true if listener was removed succesfully, false otherwise.
         */
        virtual bool removeListener(CameraListener& listener);

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
        //! the list of CameraListeners
        std::vector<CameraListener*> _listeners;

        //! name of camera model information
        std::string _modelInfo;

        //! state variable - true if camera is initialized
        bool _initialized;

        //! state variable - true if camera is started
        bool _started;

    private:
        Camera(const Camera&);
        Camera& operator=(const Camera&);
    };

    /* @} */

}} // end namespaces

#endif // end include guard
