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


#ifndef RW_MODELS_TREEDEVICE_HPP
#define RW_MODELS_TREEDEVICE_HPP

/**
 * @file TreeDevice.hpp
 */

#include "JointDevice.hpp"

#include <rw/common/Ptr.hpp>

#include <vector>

namespace rw { namespace models {
	class JacobianCalculator;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A tree structured device
     *
     * This device type defines devices that are tree-structured, with multiple end effectors.
     * Typical for dexterous hands, and multi-armed robots.
     *
     * @dot
     * digraph TreeDevice {
     *  node [shape=record, fontname=Helvetica, fontsize=10, style=filled];
     *  Base [ fillcolor="red"];
     *  Link1 [ label="Link1\n<Link>", fillcolor="red"];
     *  Axis1 [ label="Axis1\n<Joint>", fillcolor="red"];
     *  Link2 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis2 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link3 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis3 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector1 [ fillcolor="red"];
     *  Link4 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis4 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link5 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis5 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector2 [ fillcolor="red"];
     *  Link6 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis6 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector3 [ fillcolor="red"];
     *
     * world -> object1;
     * world -> table;
     * table -> object2;
     * world -> Base;
     * Base -> Link1;
     * Link1 -> Axis1;
     * Axis1 -> Link2;
     * Link2 -> Axis2;
     * Axis2 -> Link3;
     * Link3 -> Axis3;
     * Axis3 -> EndEffector1;
     * Axis1 -> Link4
     * Link4 -> Axis4
     * Axis4 -> Link5
     * Link5 -> Axis5
     * Axis5 -> EndEffector2
     * Axis2 -> Link6
     * Axis6 -> EndEffector3
     * }
     * @enddot
     *
     */
    class TreeDevice : public JointDevice
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<TreeDevice> Ptr;
        
		/**
         * @brief Constructor
         *
         * @param base [in] the base frame of the robot
         * @param ends [in] the set of end-effectors of the robot
         * @param name [in] name of device
         * @param state [in] the initial state of everything
         */
        TreeDevice(
            kinematics::Frame* base,
            const std::vector<kinematics::Frame*>& ends,
            const std::string& name,
            const kinematics::State& state);

        //! @brief destructor
        virtual ~TreeDevice();

        /**
         * @brief like Device::baseJend() but with a Jacobian calculated for all
         * end effectors.
         */
        math::Jacobian baseJends(const kinematics::State& state) const;

        /**
           @brief The end-effectors of the tree device.
         */
        const std::vector<kinematics::Frame*>& getEnds() const { return _ends; }

        /**
         * @brief Frames of the device.
         *
         * This method is being used when displaying the kinematic structure of
         * devices in RobWorkStudio. The method really isn't of much use for
         * everyday programming.
         */
        const std::vector<kinematics::Frame*>& frames() const
        { return _kinematicChain; }

    private:
        std::vector<kinematics::Frame*> _kinematicChain;
        std::vector<kinematics::Frame*> _ends;

        // Base to getEnds() Jacobians are calculated here.
        rw::common::Ptr<JacobianCalculator> _djmulti;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
