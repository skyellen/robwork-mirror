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


#ifndef RW_INVKIN_ITERATIVEMULTIIK_HPP
#define RW_INVKIN_ITERATIVEMULTIIK_HPP

/**
 * @file IterativeIK.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyMap.hpp>

#include <vector>

namespace rw { namespace kinematics { class State; } }

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    /**
     * @brief Interface for iterative inverse kinematics algorithms for problems
     * or devices that utilize more than one end-effector.
     *
     * The IterativeMultiIK interface provides an interface for calculating
     * the inverse kinematics of a device with multiple end-effectors. That is
     * to calculate
     * \f$\mathbf{q}\f$ such that \f$\robabx{base}{end}{\mathbf{T}}(\mathbf{q})=
     * \robabx{}{desired}{\mathbf{T}}\f$.
     *
     * By default it solves the problem beginning at the robot base and
     * ending with the frame defined as the end of the devices, and which is
     * accessible through the Device::getEnd() method.
     */
    class IterativeMultiIK
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<IterativeMultiIK> Ptr;

		/**
		 * @brief Destructor
		 */
        virtual ~IterativeMultiIK() {}

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
        virtual std::vector<rw::math::Q> solve(const std::vector<math::Transform3D<> >& baseTend,
                                               const kinematics::State& state) const = 0;

        /**
         * @brief Sets the maximal error for a solution
         *
         * The error between two transformations is defined as the maximum of infinite-norm
         * of the positional error and the angular error encoded as EAA.
         *
         * @param maxError [in] the maxError. It will be assumed that maxError > 0
         */
        virtual void setMaxError(const std::vector<double>& maxError);

        /**
         * @brief Returns the maximal error for a solution
         *
         * @return Maximal error
         */
        virtual std::vector<double> getMaxError() const;

        /**
         * @brief Sets the maximal number of iterations allowed
         *
         * @param maxIterations [in] maximal number of iterations
         */
        virtual void setMaxIterations(int maxIterations);

        /**
         * @brief Returns the maximal number of iterations
         */
        virtual int getMaxIterations() const;

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
        IterativeMultiIK(size_t nrOfEndEff);

    private:
        /**
         * @brief the Properties
         */
        rw::common::PropertyMap _properties;

        /**
         * @brief nr of end effectors
         */
        size_t _nrOfEndEff;

    private:
        IterativeMultiIK(const IterativeMultiIK&);
        IterativeMultiIK& operator=(const IterativeMultiIK&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
