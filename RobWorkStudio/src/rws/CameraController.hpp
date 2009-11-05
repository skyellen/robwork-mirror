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

#ifndef CAMERACONTROLLER_HPP_
#define CAMERACONTROLLER_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/common/Ptr.hpp>

/**
 * @brief an interface for controlling the camera using a mouse.
 */
class CameraController {

public:
    /**
     * @brief destructor
     */
    virtual ~CameraController() { };

    /**
     * @brief set the bounds that define the area where the 2d point
     * is valid. The bound is defined in a plane with [0,width] and
     * [0,height]. Where (0,0) is the upper left corner of the plane.
     * @param NewWidth [in] width
     * @param NewHeight [in] height
     */
    virtual void setBounds(float NewWidth, float NewHeight) = 0;

    /**
     * @brief Sets the 2d center of the ArcBall sphere.
     * @param x [in] the x-coodinate of the center point
     * @param y [in] the y-coodinate of the center point
     */
    virtual void setCenter(float x, float y) = 0;

    /**
     * @brief register a mouse click event. The coordinates must be inside the
     * specified bounds.
     * @param x [in] x-coodinate
     * @param y [in] y-coodinate
     */
    virtual void click(float x, float y) = 0;

    /**
     * @brief Calculates the rotation of the object/scene based on
     * the mouse being dragged to the position (x,y).
     * @param x [in] the x-coordinate of the current mouse position
     * @param y [in] the y-coordinate of the current mouse position
     * @return the rotation that should be applied to the object/scene
     */
    virtual rw::math::Quaternion<float> drag(float x, float y) = 0;

    /**
     * @brief draw the camera control.
     */
    virtual void draw() = 0;

};

typedef rw::common::Ptr<CameraController> CameraControllerPtr;

#endif /* CAMERACONTROLLER_HPP_ */
