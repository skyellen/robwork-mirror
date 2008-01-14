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

#ifndef rw_models_SerialDevice_HPP
#define rw_models_SerialDevice_HPP

/**
 * @file SerialDevice.hpp
 */

#include "JointDevice.hpp"

#include <rw/math/Transform3D.hpp>
#include <vector>

namespace rw { namespace models {

    class Joint;

    /** @addtogroup models */
    /*@{*/

    /**
       @brief The device for a serial chain.

       SerialChain is like JointDevice except that SerialChain has the
       additional guarantee that the joints lie on a single parent to child path
       of the kinematic tree.
     */
    class SerialDevice : public JointDevice
    {
    public:
        /**
         * @brief Constructor
         *
         * @param first [in] the base frame of the robot
         * @param last [in] the end-effector of the robot
         * @param name [in] name of device
         * @param state [in] the connectedness of the frames
         */
        SerialDevice(
            kinematics::Frame* first,
            kinematics::Frame* last,
            const std::string& name,
            const kinematics::State& state);

        /**
         * @brief Frames of the device.
         *
         * This method is being used when displaying the kinematic structure of
         * devices in RobWorkStudio. The method really isn't of much use for
         * everyday programming.
         */
        const std::vector<kinematics::Frame*>& frames() const;

        /**
         * @brief Creates object
         *
         * @param serialChain [in] a vector of connected frames. The first frame
         * in \b serialChain is the base of the device and the last frame of \b
         * serialChain is the end of the device. The joints of the device are
         * the active joints of \b serialChain.
         *
         * @param name [in] name of device
         *
         * @param state [in] the initial state of everything
         */
        SerialDevice(
            const std::vector<kinematics::Frame*>& serialChain,
            const std::string& name,
            const kinematics::State& state);

    private:
        std::vector<kinematics::Frame*> _kinematicChain;
    };

    /*@}*/
}} // end namespaces

/*
 * @example workcell/PUMA560.cpp
 *
 * This is an example of how to use the SerialDevice class to model a robot
 *
 * The example models an Unimation PUMA 560 robot and calculates its forward
 * kinematics at 2 different configurations
 */

#endif // end include guard
