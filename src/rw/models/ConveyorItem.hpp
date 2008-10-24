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
