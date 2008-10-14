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

#ifndef rw_inversekinematics_ClosedFormIK_HPP
#define rw_inversekinematics_ClosedFormIK_HPP

/**
 * @file ClosedFormIK.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/Device.hpp>

#include <vector>

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    class ClosedFormIK;

    //! A pointer to a ClosedFormIK solver.
    typedef rw::common::Ptr<ClosedFormIK> ClosedFormIKPtr;

    /**
     * @brief Interface for closed form inverse kinematics algorithms.
     *
     * The ClosedFormIK interface provides an interface for calculating the
     * inverse kinematics of a device. That is to calculate the solutions
     * \f$\mathbf{q}_i, i=0,\ldots,\f$, such that
     * \f$\robabx{base}{end}{\mathbf{T}}(\mathbf{q}_i)=
     * \robabx{}{desired}{\mathbf{T}}\f$.
     *
     * By default it solves the problem beginning at the robot base and
     * ending with the frame defined as the end of the devices, and which is
     * accessible through the Device::getEnd() method.
     */
    class ClosedFormIK
    {
    public:
        /**
         * @brief Calculates the inverse kinematics solutions
         *
         * Given a desired \f$\robabx{}{desired}{\mathbf{T}}{}\f$ the method
         * solves the inverse kinematics problem, and provides a list of valid
         * configuration. That is only configurations which are within the
         * bounds of the configuration space.
         *
         * @param baseTend [in] Desired base to end transformation \f$
         * \robabx{}{desired}{\mathbf{T}}\f$
         *
         * @return List of valid solutions. Notice that the list may be empty.
         */
        virtual std::vector<math::Q> solve(
            rw::math::Transform3D<>& baseTend) const = 0;

        /**
           @brief Closed-form IK solver for a device.

           The device must be a serial device with 6 revolute joints described
           by DH parameters.

           The IK solver is currently implemented in terms of PieperSolver. See
           the documentation of PieperSolver for the specific requirements for
           the DH parameters.

           An exception is thrown if closed-form IK for the device is not
           supported, except that all such cases are currently not discovered.
           You should check for yourself that the closed-form IK for the device
           is correct.
        */
        static
        ClosedFormIKPtr make(
            const rw::models::Device& device,
            const rw::kinematics::State& state);

        /**
           @brief Destructor
        */
        virtual ~ClosedFormIK() {}

    protected:
        /**
           @brief Constructor
        */
        ClosedFormIK() {}

    private:
        ClosedFormIK(const ClosedFormIK&);
        ClosedFormIK& operator=(const ClosedFormIK&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
