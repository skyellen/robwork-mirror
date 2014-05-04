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


#ifndef RW_SENSOR_CAMERAMODEL_HPP
#define RW_SENSOR_CAMERAMODEL_HPP

/**
 * @file CameraModel.hpp
 */

#include "Image.hpp"
#include "SensorModel.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>

#include <vector>
#include <string>

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief The CameraModel class defines a generel pinhole camera model where
     * camera parameters and state values are stored.
     *
     */
    class CameraModel : public SensorModel
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
        CameraModel(rw::math::ProjectionMatrix projection,
            const std::string& name,
            const std::string& modelInfo);

        /**
         * @brief destructor
         */
        virtual ~CameraModel();


        /**
         * @brief returns the image if it has been saved in the State. Else null is
         * returned.
         * @return last image captured from camera.
         */
        Image::Ptr getImage(rw::kinematics::State& state);

        /**
         * @brief set the image in the state
         * @param img [in] image to set in state
         * @param state [in/out] the state in which to set the image.
         */
        void setImage(Image::Ptr img, rw::kinematics::State& state);

        //! get the camera projection matrix
        rw::math::ProjectionMatrix getProjectionMatrix();

        //! field of view of camera
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

        double getFarClippingPlane();
        double getNearClippingPlane();


    protected:

        //! name of camera model information
        std::string _modelInfo;

        rw::math::ProjectionMatrix _pmatrix;


    };

    /* @} */

}} // end namespaces

#endif // end include guard
