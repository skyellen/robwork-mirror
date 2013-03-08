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


#ifndef RW_SENSOR_CAMERADRIVER_HPP
#define RW_SENSOR_CAMERADRIVER_HPP

/**
 * @file Camera.hpp
 */

#include "Image.hpp"
#include "Sensor.hpp"
#include "CameraListener.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>

#include <vector>
#include <string>

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief The CameraModel class defines a generel kinematic camera model where
     * camera parameters and state values are stored.
     *
     */
    class CameraModel : public Sensor
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<CameraModel> Ptr;

    public:
        /**
         * @brief constructor
         * @param name [in] name of sensor
         * @param modelInfo [in] info string
         */
        CameraModel(
            const std::string& name,
            const std::string& modelInfo);

        /**
         * @brief sets the camera model information
         * @param info [in] information of the camera
         */
        void setModelInfo(const std::string info) { _modelInfo = info; }

        /**
         * @brief destructor
         */
        virtual ~CameraModel();

        /**
         * @brief returns the camera model information (version, type, size, etc.)
         * @return camera model information
         */
        virtual std::string getModelInfo() const { return _modelInfo;};

        /**
         * @brief returns the last image acquired from the camera. This method
         * is not blocking, if no image has been acquired yet an empty image
         * is returned. The image returned can for some specific drivers be read
         * only.
         * @return last image captured from camera.
         */
        Image::Ptr getImage(rw::kinematics::State& state);

        /**
         * @brief image
         * @param img
         * @param state
         */
        void setImage(Image::Ptr img, rw::kinematics::State& state);


        double getFocalLength();


        //!@brief get horisontal field of view in degrees.
        double getFieldOfViewX();

        //!@brief get vertical field of view in degrees.
        double getFieldOfViewY();


        /**
         * @brief get width of the captured images
         * @return width
         */
        unsigned int getWidth();

        /**
         * @brief set width of this camera model
         */
        void setWidth(unsigned int);

        /**
         * @brief get width of the captured images
         * @return width
         */
        unsigned int getHeight();


        void setHeight(unsigned int width);

        ///// a list of features that most of the time is available


    protected:

        //! name of camera model information
        std::string _modelInfo;

        bool _hasGain, _hasShutter;
        double _gain;
        double _shutter;
    };

    /* @} */

}} // end namespaces

#endif // end include guard
