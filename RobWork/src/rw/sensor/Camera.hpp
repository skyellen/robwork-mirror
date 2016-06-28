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

#include <rw/common/Ptr.hpp>

#include <vector>
#include <string>

namespace rw { namespace sensor {

	class CameraListener;

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
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<Camera> Ptr;

    protected:
        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param modelInfo [in] info string
         */
        Camera(
            const std::string& name,
            const std::string& modelInfo);

        /**
         * @brief sets the camera model information
         * @param info [in] information of the camera
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
         * @brief get width of the captured images
         * @return width
         */
        virtual unsigned int getWidth() = 0;

        /**
         * @brief get width of the captured images
         * @return width
         */
        virtual unsigned int getHeight() = 0;

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


        ///// a list of features that most of the time is available

        /**
         *  Check if shutter is available.
         *  @return True if shutter is available
         */
        virtual bool isShutterAvailable() const{return false;};

        /**
         * Get actual shutter value.
         * Note: If shutter is not available then a dummy implementation
         * will throw an error message.
         * @return shutter value in micro-seconds.
         */
        virtual double getShutter() const{
            RW_THROW("Shutter not available!");
            return -1.0;
        };

        /**
         * Set shutter value. If the given value is not possible the nearest
         * value are choosen.
         * Note: If shutter is not available then a dummy implementation
         * will throw an error message.
         * @param Value New shutter value.
         * @return New nearest shutter value.
         */
        virtual void setShutter(double Value){
            RW_THROW("Shutter not available!");
        };

        /**
         * gets the shutter bounds.
         * Note: If shutter is not available then a dummy implementation
         * will throw an error message.
         * @return first value is the min bound and second value is the max bound
         */
        virtual std::pair<double,double> getShutterBounds() const{
            RW_THROW("Shutter not available!");
            return std::pair<double,double>(-1.0,-1.0);
        };

        /**
         * Check if gain is available.
         * @return True if zoom is available
         */
        virtual bool isGainAvailable() const{return false;};

        /**
         * Get actual gain value.
         * Note: If gain is not available then a dummy implementation
         *  returning -1 is used and an error message is produced.
         *  @return Gain value.
         */
        virtual double getGain() const{
            RW_THROW("Gain not available!");
            return -1.0;
        };

        /** Set gain value. If the given value is not possible the nearest
            value are choosen.
            Note: If gain is not available then a dummy implementation
            returning -1 is used and an error message is produced.
            @param Value New gain value.
            @return New nearest gain value. */
        virtual double setGain(double Value){
            RW_THROW("Gain not available!");
            return -1.0;
        };


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

#ifdef RW_USE_DEPRECATED
    /**
     * @brief Smart pointer to Camera
     */
    typedef rw::common::Ptr<Camera> CameraPtr;
#endif
}} // end namespaces

#endif // end include guard
