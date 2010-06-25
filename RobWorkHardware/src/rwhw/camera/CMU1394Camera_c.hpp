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

#ifndef RWHW_CMU1394CAMERA_HPP
#define RWHW_CMU1394CAMERA_HPP

/**
 * @file CMU1394Camera_c.hpp
 */

#include <rw/sensor/CameraFirewire.hpp>
#include <rw/sensor/Image.hpp>

#include <vector>

#include <1394Camera_c.h>

namespace rwhw {
    /** @addtogroup camera */
    /* @{ */

    /**
     * @brief This class implements the Camera interface using
     * the CMU 1394 windows drivers. Check out http://www.cs.cmu.edu/~iwan/1394/
     */
    class CMU1394CameraC : public rw::sensor::CameraFirewire
    {
    protected:
        /**
         * @brief constructor
         * @param cam [in] handle to camera
         */
        CMU1394CameraC(CameraID camid,std::string camName, std::string vendorName);

        /**
         * @brief returns the C1394Camera object that is bound to this WinCamera
         * @return C1394Camera object that is bound to this WinCamera
         */
        CameraID getCMUCamera(){return _camID;};

        /**
         * @brief if a C1394Camera class is connected to this Camera class
         * then true is returned, false otherwise
         */
        bool isConnected(){return _connected;};

        /**
         * @brief sets the connected state of this WinCamera
         * @param connected [in] true to set the state of WinCamera to connected,
         * false otherwise
         */
        void setConnected(bool connected){_connected = connected;};

    public:

        /**
         * @brief destructor
         */
        virtual ~CMU1394CameraC();

        /**
         * @brief return handles (Camera objects) to all connected
         * firewire cameras.
         * @return a list of available cameras
         */
        static const std::vector<CMU1394CameraC*> getCameraHandles();

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

        const rw::sensor::Image* grab();

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
         * @copydoc rw::sensor::CameraFirewire::getCaptureMode
         */
        virtual CaptureMode getCaptureMode();

        /**
         * @copydoc rw::sensor::CameraFirewire::setCaptureMode
         */
        virtual bool setCaptureMode(CaptureMode mode);

        /**
         * @copydoc rw::sensor::Camera::getWidth
         */
        unsigned int getWidth(){ return _width;};

        /**
         * @copydoc rw::sensor::Camera::getHeight
         */
        unsigned int getHeight(){return _height;};


        /**
         * @copydoc rw::sensor::Camera::getCapturePolicy
         */
        CapturePolicy getCapturePolicy();

        void close();

        /**
         * @copydoc rw::sensor::Camera::isShutterAvailable
         */
        virtual bool isShutterAvailable() const;

        /**
         * @copydoc rw::sensor::Camera::getShutter
         */
        virtual double getShutter() const;

        /**
         * @copydoc rw::sensor::Camera::setShutter
         */
        virtual void setShutter(double Value);

        virtual std::pair<double,double> getShutterBounds() const;

        //bool isStarted(){ return _started; };
    protected:
        bool setWithCurrentValues();
        bool selectCameraFromID();

    private:
        bool _connected;
        // List of all cameras
        static std::vector<CameraID> _cameras;
        static std::vector<CMU1394CameraC*> _connectedCameras;
        static bool _queryCameras;
        CapturePolicy _policy;
        bool _isAquired;
        CameraID _camID;
        rw::sensor::Image *_image;
        CameraFirewire::CaptureMode _captureMode;
        double _frameRate;
        LARGE_INTEGER _uniqueID;

        static const std::string sFrameRates[];
        static const double FrameRates[];

        unsigned int _width, _height;

        //bool _started;
    };
    /* @} */

} // end namespaces

#endif // end include guard
