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

#ifndef RW_KINEMATICS_MOVABLEFRAME_HPP
#define RW_KINEMATICS_MOVABLEFRAME_HPP

/**
 * @file MovableFrame.hpp
 */

#include "Frame.hpp"
#include "State.hpp"

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /* @{ */

    /**
     * @brief MovableFrame is a frame for which it is possible to freely
     * change the transform relative to the parent.
     *
     * A MovableFrame can for example be used for modelling objects moving in
     * the scene based on e.g. user input.
     */
    class MovableFrame: public Frame
    {
    public:
        /**
         * @brief Construct a MovableFrame with Identiy as the initial
         * transform
         *
         * @param name [in] name of the frame
         */
        explicit MovableFrame(const std::string& name);

        /**
         * @copydoc Frame::getTransform
         */
        math::Transform3D<> getTransform(const State& state) const;

        /**
         * @brief Sets the transform in the state
         * @param transform [in] transform to set
         * @param state [out] state into which to set the transform
         */
        void setTransform(const math::Transform3D<>& transform, State& state);

    private:
        void doGetTransform(const math::Transform3D<>& parent,
                            const State& state,
                            math::Transform3D<>& result) const;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
