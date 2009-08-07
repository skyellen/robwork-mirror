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


#ifndef RW_TRAJECTORY_PATH_HPP
#define RW_TRAJECTORY_PATH_HPP

/**
   @file Path.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <vector>
#include "Timed.hpp"

namespace rw { namespace trajectory {
    /**
     *  @brief std::vector of rw::math::Q
     */
    typedef std::vector<rw::math::Q> QPath;

    /**
     * @brief Pointer to QPath
     */
    typedef rw::common::Ptr<QPath> QPathPtr;


    /**
     * @brief std::vector of rw::math::Vector3D<>
     */
    typedef std::vector<rw::math::Vector3D<> > Vector3DPath;

    /**
     * @brief Pointer to Vector3DPath
     */
    typedef rw::common::Ptr<Vector3DPath> Vector3DPathPtr;


    /**
     * @brief std::vector of rw::math::Rotation3D<>
     */
    typedef std::vector<rw::math::Rotation3D<> > Rotation3DPath;

    /**
     * @brief Pointer to Rotation3DPath
     */
    typedef rw::common::Ptr<Rotation3DPath> Rotation3DPathPtr;


    /**
     * @brief std::vector of rw::math::Transform3D<>
     */
    typedef std::vector<rw::math::Transform3D<> > Transform3DPath;

    /**
     * @brief Pointer to Transform3DPath
     */
    typedef rw::common::Ptr<Transform3DPath> Transform3DPathPtr;


    /**
     *  @brief std::vector of rw::kinematics::State
     */
    typedef std::vector<rw::kinematics::State> StatePath;

    /**
     * @brief Pointer to StatePath
     */
    typedef rw::common::Ptr<StatePath> StatePathPtr;



    /**
       @brief std::vector of rw::math::Q with associated times
    */
    typedef std::vector<TimedQ> TimedQPath;

    /**
     * @brief Pointer to TimedQPath
     */
    typedef rw::common::Ptr<TimedQPath> TimedQPathPtr;


    /**
       @brief std::vector of rw::kinematics::State with associated times
    */
    typedef std::vector<TimedState> TimedStatePath;

    /**
     * @brief Pointer to TimedStatePath
     */
    typedef rw::common::Ptr<TimedStatePath> TimedStatePathPtr;


}} // end namespaces

#endif // end include guard
