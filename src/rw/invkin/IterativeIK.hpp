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

#ifndef rw_inversekinematics_IterativeIK_HPP
#define rw_inversekinematics_IterativeIK_HPP

/**
 * @file IterativeIK.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyMap.hpp>

#include <vector>

namespace rw { namespace inversekinematics {

    /** \addtogroup inversekinematics */
    /*@{*/
    
    /**
     * @brief Interface for iterative inverse kinematics algorithms
     *
     * The IterativeIK interface provides an interface for calculating
     * the inverse kinematics of a device. That is to calculate
     * \f$\mathbf{q}\f$ such that \f$\robabx{base}{end}{\mathbf{T}}(\mathbf{q})=
     * \robabx{}{desired}{\mathbf{T}}\f$.
     *
     * By default it solves the problem beginning at the robot base and
     * ending with the frame defined as the end of the devices, and which is
     * accessible through the Device::getEnd() method.
     */
    class IterativeIK
    {
    public:
        virtual ~IterativeIK() {}

        /**
         * @brief Calculates the inverse kinematics
         *
         * Given a desired \f$\robabx{}{desired}{\mathbf{T}}\f$
         * and the current state, the method solves the inverse kinematics
         * problem. If no solution is found with the required precision and
         * within the specified number of iterations it will return an empty
         * list.
         *
         * If the algorithm is able to identify multiple solutions (e.g. elbow
         * up and down) it will return all of these. Before returning a solution,
         * it is checked to be within the bounds of the configuration space.
         *
         * @param baseTend [in] Desired base to end transformation \f$
         * \robabx{}{desired}{\mathbf{T}}\f$
         *
         * @param state [in] State of the device from which to start the
         * iterations
         *
         * @return List of solutions. Notice that the list may be empty.
         */
        virtual std::vector<rw::math::Q> solve(
            const math::Transform3D<>& baseTend,
            const kinematics::State& state) const = 0;

        /**
         * @brief Sets the maximal error for a solution
         *
         * The error between two transformations is defined as the maximum of infinite-norm
         * of the positional error and the angular error encoded as EAA.
         *
         * @param maxError [in] the maxError. It will be assumed that maxError > 0
         */
        virtual void setMaxError(double maxError);

        /**
         * @brief Returns the maximal error for a solution
         *
         * @return Maximal error
         */
        virtual double getMaxError() const;

        /**
         * @brief Sets the maximal number of iterations allowed
         *
         * @param maxIterations [in] maximal number of iterations
         */
        virtual void setMaxIterations(unsigned int maxIterations);

        /**
         * @brief Returns the maximal number of iterations
         */
        virtual unsigned int getMaxIterations() const;

        /**
         * @brief Returns the PropertyMap
         * @return Reference to the PropertyMap
         */
        virtual rw::common::PropertyMap& getProperties();

        /**
         * @brief Returns the PropertyMap
         * return Reference to the PropertyMap
         */
        virtual const rw::common::PropertyMap& getProperties() const;

    protected:
        /**
         * @brief Constructor
         */
        IterativeIK();

        /**
         * @brief the Properties
         */
        rw::common::PropertyMap _properties;
    };
    /*@}*/
}} // end namespaces

#endif // end include guard
