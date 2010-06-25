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

#ifndef RWHW_OPENCVCAMERA_HPP
#define RWHW_OPENCVCAMERA_HPP

#include <cv.h>
#include <highgui.h>


/**
 * @file CMU1394Camera.hpp
 */

#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Image.hpp>

#include <vector>

namespace rwhw {
    /** @addtogroup camera */
    /* @{ */

	class OpenCVCamera;

    /**
     * @brief This class implements the Camera interface using
     * the CMU 1394 windows drivers. Check out http://www.cs.cmu.edu/~iwan/1394/
     */
    class OpenCVCamera : public rw::sensor::Camera
    {
    protected:
        /**
         * @brief constructor
         * @param cam [in] handle to camera
         */
        OpenCVCamera(CvCapture* cam);

        /**
         * @brief returns the C1394Camera object that is bound to this WinCamera
         * @return C1394Camera object that is bound to this WinCamera
         */
        CvCapture* getCamera();

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
        virtual ~OpenCVCamera();

        /**
         * @brief return handles (Camera objects) to all connected
         * firewire cameras.
         * @return a list of available cameras
         */
        static const std::vector<OpenCVCamera*> getCameraHandles();

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
         * @copydoc rw::sensor::Camera::getWidth
         */
        unsigned int getWidth(){ return _width;};

        /**
         * @copydoc rw::sensor::Camera::getHeight
         */
        unsigned int getHeight(){return _height;};
    private:
        bool _connected;
        // List of all cameras
        static std::vector<OpenCVCamera*> _cameras;

        bool _isAquired;
        CvCapture* _cam;

        rw::sensor::Image *_image;

        double _frameRate;

        unsigned int _width, _height;
    };
    /* @} */

} // end namespaces

#endif // end include guard
