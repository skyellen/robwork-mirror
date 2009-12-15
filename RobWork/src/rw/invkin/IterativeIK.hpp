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


#ifndef RW_INVKIN_ITERATIVEIK_HPP
#define RW_INVKIN_ITERATIVEIK_HPP

/**
 * @file IterativeIK.hpp
 */

#include "InvKinSolver.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyMap.hpp>

#include <vector>

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    class IterativeIK;

    //! A pointer to a IterativeIK solver.
    typedef rw::common::Ptr<IterativeIK> IterativeIKPtr;

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
    class IterativeIK: public InvKinSolver
    {
    public:
        virtual ~IterativeIK() {}

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

        /**
           @brief Default iterative inverse kinematics solver for a device and
           state.

           @param device [in] Device for which to solve IK.
           @param state [in] Fixed state for which IK is solved.
        */
        static IterativeIKPtr makeDefault(rw::models::DevicePtr device,
                                          const rw::kinematics::State& state);

    protected:
        /**
         * @brief Constructor
         */
        IterativeIK();

    private:
        /**
         * @brief the Properties
         */
        rw::common::PropertyMap _properties;

    private:
        IterativeIK(const IterativeIK&);
        IterativeIK& operator=(const IterativeIK&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
