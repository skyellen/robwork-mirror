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


#ifndef RW_MODELS_CONVEYORITEM_HPP
#define RW_MODELS_CONVEYORITEM_HPP

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

namespace rw { namespace models {

    /**
     * @brief An item which can be placed on a Conveyor
     *
     * The ConveyorItem inherits from Frame and provides and item which can be placed an a
     * Conveyor. Either use the ConveyorItem directly or use it as a parent frame for objects
     * which should be moved.
     *
     * The ConveyorItem is attached as a DAF, hence its parent depends on the State
     */
    class ConveyorItem: public kinematics::Frame
    {
    public:
        /**
         * @brief Constructor
         * @param name [in] The name
         */
        ConveyorItem(const std::string& name);

        /**
         * @brief Destructor
         */
        virtual ~ConveyorItem();

        /**
         * @copydoc rw::kinematics::Frame::getTransform
         */
        rw::math::Transform3D<> getTransform(const rw::kinematics::State& state) const;

        /**
         * @brief Sets the transform and the conveyor position of the item
         *
         * Only to be used from within a ConveyorSegment!
         */
        void setTransformAndConveyorPosition(
            const rw::math::Transform3D<>& pose,
            double q,
            rw::kinematics::State& state) const;

        /**
         * @brief Returns the position of the item in the Conveyor
         *
         * Only to be used from within a ConveyorSegment!
         */
        double getConveyorPosition(const rw::kinematics::State& state) const;

    private:
        void doGetTransform(
            const math::Transform3D<>& parent,
            const kinematics::State& state,
            math::Transform3D<>& result) const;
    };

}} // end namespaces

#endif //#ifndef RW_MODELS_CONVEYORITEM_HPP
