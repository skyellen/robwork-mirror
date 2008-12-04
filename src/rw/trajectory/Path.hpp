/*********************************************************************
 * RobWork Version 0.3
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
