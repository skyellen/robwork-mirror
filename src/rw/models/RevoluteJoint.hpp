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

#ifndef RW_MODELS_REVOLUTEJOINT_HPP
#define RW_MODELS_REVOLUTEJOINT_HPP

/**
 * @file RevoluteJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Revolute joints.

       RevoluteJoint implements a revolute joint for the rotation about the
       z-axis of an arbitrary displacement transform.
    */
    class RevoluteJoint : public Joint
    {
    public:

        RevoluteJoint(const std::string& name,
                      const math::Transform3D<>& transform);

        /**
           @brief A revolute joint with a displacement transform of \b transform.

           @param name [in] The name of the frame.
           @param transform [in] The displacement transform of the joint.
        */
       /* static
        RevoluteJoint* make(
            const std::string& name,
            const math::Transform3D<>& transform);*/

        /// @cond SHOW_ALL
        /**
           @brief The transform of the joint for a given joint value.

           This method is useful for passive joints where the joint value \b q
           is computed from a combination of other frame values of the state.
        */
       /* void getJointValueTransform(
            const math::Transform3D<>& parent,
            double q,
            math::Transform3D<>& result) const;*/
        /// @endcond

        class RevoluteJointImpl;

        void multiplyJointTransform(const math::Transform3D<>& parent,
                                      const math::Q& q,
                                      math::Transform3D<>& result) const;

        math::Transform3D<> getJointTransform(const math::Q& q) const;



        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;

    protected:




        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;

        math::Transform3D<> doGetTransform(const kinematics::State& state) const;

/*

        virtual void doGetJointValueTransform(const math::Transform3D<>& parent,
                                              double q,
                                              math::Transform3D<>& result) const = 0;

*/
    private:
        RevoluteJointImpl* _impl;


    protected:
        /**
           @brief Subclasses should call this constructor.
        */
    /*    explicit RevoluteJoint(const std::string& name) :
            Joint(name)
        {}

    private:
        void doGetTransform(
            const math::Transform3D<>& parent,
            const kinematics::State& state,
            math::Transform3D<>& result) const;

        virtual void doGetJointValueTransform(
            const math::Transform3D<>& parent,
            double q,
            math::Transform3D<>& result) const = 0;*/
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
