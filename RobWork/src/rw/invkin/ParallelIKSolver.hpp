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


#ifndef RW_INVKIN_PARALLELIKSOLVER_HPP
#define RW_INVKIN_PARALLELIKSOLVER_HPP

/**
 * @file ParallelIKSolver.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/common/PropertyMap.hpp>
#include "IterativeIK.hpp"

namespace rw { namespace models { class ParallelDevice; }}

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * \brief This inverse kinematics method is a heuristic search technique
     */
    class ParallelIKSolver : public IterativeIK
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ParallelIKSolver> Ptr;


        /**
         * @brief Constructor
         */
        ParallelIKSolver(const models::ParallelDevice* device);


		/**
		 * @brief Destructor
		 */
        virtual ~ParallelIKSolver();

        /**
         * \copydoc IterativeIK::solve
         */
        virtual std::vector<math::Q> solve(const math::Transform3D<>& baseTend,
                                           const kinematics::State &state) const;

    private:
        typedef std::pair<rw::math::Vector3D<double>, rw::math::Quaternion<double> > QPose;

		typedef rw::math::Q::BoostVector BoostVector;

        const models::ParallelDevice* _device;
        rw::common::PropertyMap _properties;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
