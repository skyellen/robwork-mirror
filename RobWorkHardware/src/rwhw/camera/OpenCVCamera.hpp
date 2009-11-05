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
    /** @addtogroup rwhw */
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
