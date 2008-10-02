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

#ifndef rw_models_PassiveRevoluteFrame_HPP
#define rw_models_PassiveRevoluteFrame_HPP

/**
 * @file PassiveRevoluteFrame.hpp
 */

#include "Joint.hpp"
#include "RevoluteJoint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Passive revolute joints.
     *
     * PassiveRevoluteFrame implements a revolute joint for the rotation about the
     * z-axis of an arbitrary displacement transform.
     */
    class PassiveRevoluteFrame : public kinematics::Frame
    {
    public:
        /**
         * @brief A revolute joint with a displacement transform of \b transform.
         *
         * @param parent [in] The parent frame
         *
         * @param name [in] The name of the frame.
         *
         * @param transform [in] The displacement transform of the joint.
         *
         * @param owner [in] The joint controlling the passive joint.
         *
         * @param scale [in] Scaling factor for the controlling joint value.
         *
         * @param offset [in] Offset for the controlling joint value.
         */
        PassiveRevoluteFrame(
            const std::string& name,
            const math::Transform3D<>& transform,
            Joint* owner,
            double scale,
            double offset);

        /**
         * @brief The parent to frame transform for a revolute joint.
         *
         * The parent to frame transform is T * Rz(q) where:
         *
         * - T is the displacement transform of the joint;
         *
         * - q = q_owner * scale + offset is the joint value of the joint;
         *
         * - Rz(q) is the transform that rotates a point an angle q about the
         * z-axis.
         *
         * @copydoc kinematics::Frame::getTransform
         */
        math::Transform3D<> getTransform(const kinematics::State& state) const;

        /**
           @brief The joint controlling the passive revolute frame.
        */
        const Joint& getOwner() const { return *_owner; }

        /**
           @brief The joint controlling the passive revolute frame.
        */
        Joint& getOwner() { return *_owner; }

        /**
           @brief The scaling factor for the joint value of the controlling joint.
         */
        double getScale() const { return _scale; }

    private:
        void doGetTransform(
            const math::Transform3D<>& parent,
            const kinematics::State& state,
            math::Transform3D<>& result) const;

    private:
        RevoluteJoint _helper;
        Joint* _owner;
        double _scale;
        double _offset;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
