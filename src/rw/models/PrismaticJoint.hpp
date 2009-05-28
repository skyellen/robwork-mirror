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

#ifndef RW_MODELS_PRISMATICJOINT_HPP
#define RW_MODELS_PRISMATICJOINT_HPP

/**
 * @file PrismaticJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/


    /**
     * @brief Prismatic joints.
     *
     * PrismaticJoint implements a prismatic joint for the displacement in the
     * direction of the z-axis of an arbitrary displacement transform.
     */
    class PrismaticJoint : public Joint
    {
    public:
        PrismaticJoint(const std::string& name, const math::Transform3D<>& transform);

        /**
           @brief A prismatic joint with a displacement transform of \b
           transform.

           @param name [in] The name of the frame.
           @param transform [in] The displacement transform of the joint.
        */
        /*static PrismaticJoint* make(const std::string& name,
                                    const math::Transform3D<>& transform);
*/
        /**
         @brief The transform of the joint for a given joint value.
         */
        /*void getJointValueTransform(const math::Transform3D<>& parent,
                                    double q,
                                    math::Transform3D<>& result) const;
*/
    //protected:
        /**
           @brief Subclasses should call this constructor.
        */
        /*explicit PrismaticJoint(const std::string& name) :
            Joint(name)
        {}*/

        void multiplyJointTransform(const math::Transform3D<>& parent,
                                    const math::Q& q,
                                    math::Transform3D<>& result) const;

        math::Transform3D<> getJointTransform(const math::Q& q) const;


        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;

        class PrismaticJointImpl;

    protected:
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;

        math::Transform3D<> doGetTransform(const kinematics::State& state) const;


        /*virtual void doGetJointValueTransform(const math::Transform3D<>& parent,
                                              double q,
                                              math::Transform3D<>& result) const = 0;*/



    private:
        PrismaticJointImpl* _impl;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
