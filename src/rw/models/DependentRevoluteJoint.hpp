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

#ifndef RW_MODELS_PASSIVEREVOLUTEFRAME_HPP
#define RW_MODELS_PASSIVEREVOLUTEFRAME_HPP

/**
 * @file PassiveRevoluteFrame.hpp
 */

#include "DependentJoint.hpp"
#include "RevoluteJoint.hpp"
#include <memory>

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Dependent revolute joints.
     *
     * DependentRevoluteJoint implements a revolute joint for which the rotation about the
     * z-axis are linearly dependent on another joint.
     *
     *  The parent to frame transform is T * Rz(q) where:
     *
     * - T is the displacement transform of the joint;
     *
     * - q = q_owner * scale + offset is the joint value of the joint;
     *
     * - Rz(q) is the transform that rotates a point an angle q about the
     * z-axis.
     */
    class DependentRevoluteJoint : public DependentJoint
    {
    public:
        /**
         * @brief A revolute joint with a displacement transform of \b transform.
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
        DependentRevoluteJoint(const std::string& name,
                               const math::Transform3D<>& transform,
                               Joint* owner,
                               double scale,
                               double offset);


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

        /**
         * @copydoc DependentJoint::isControlledBy
         */
        bool isControlledBy(const Joint* joint) const {
            return _owner == joint;
        }


        /**
         * @copydoc Joint::getJacobian
         */
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;

    private:
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;

        math::Transform3D<> doGetTransform(const kinematics::State& state) const;


        virtual math::Jacobian doGetJacobian(const kinematics::State& state) const;

    private:
        RevoluteJoint _helper;
        Joint* _owner;
        double _scale;
        double _offset;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
