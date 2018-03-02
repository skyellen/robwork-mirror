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


#ifndef RW_PATHPLANNING_QEdgeConstraint_HPP
#define RW_PATHPLANNING_QEdgeConstraint_HPP

/**
   @file QEdgeConstraint.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Metric.hpp>

namespace rw { namespace models { class Device; } }

namespace rw { namespace pathplanning {
	class QConstraint;

    /** @addtogroup pathplanning */
    /*@{*/

    /**
       @brief Edge constraint interface.

       An edge constraint represents a path that connects a pair of
       configurations and checks if this path can be traversed.

       The edge constraint may assume that the start and end configurations are
       valid (e.g. not colliding).

       Each edge has a non-negative cost measuring the degree to which the path
       connecting the configurations has been verified. You can use the cost
       measure to for example always verify the edge for which the most of the
       path still remains to be verified. The exact meaning of the cost is
       defined by the specific subclass.

       Given an edge constraint you can construct a new edge constraint of the same
       type, but for a new pair of configurations, with
       QEdgeConstraint::instance().
    */
    class QEdgeConstraint
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QEdgeConstraint> Ptr;
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr< const QEdgeConstraint > CPtr;

        /**
           @brief Destructor
        */
        virtual ~QEdgeConstraint();

        /**
           @brief True if the path from \b start to \b end can't be traversed.

           @param start [in] Start configuration.
           @param end [in] End configuration.
        */
        bool inCollision(const rw::math::Q& start, const rw::math::Q& end) const {
			return doInCollision(start, end);
		}

        /**
           @brief Discrete path verification for a linearly interpolated path.

           Performs a binary style checking of the edge with a resolution of \b resolution.
		   The length of the edge is virtually extended to exactly match the specified resolution.
		   However, only configurations within the original length are tested.

		   Each configuration tested is checked using \b constraint.

		   The metric must be well-behaved, i.e. linear.

           Start and end configurations are assumed to be collision free.
           
		   \param constraint [in] Constraint to check configurations with
		   \param metric [in] Metric with which the resolution it to be measured
		   \param resolution [in] The test resolution
        */
		static QEdgeConstraint::Ptr make(rw::common::Ptr<QConstraint> constraint,
										 rw::math::QMetric::CPtr metric,
										 double resolution);

        /**
           @brief Default edge constraint for a configuration constraint and a
           device.

           Start and end configurations are connected by a straight line in the
           configuration space and are checked by a default collision checking
           resolution.
        */
		static QEdgeConstraint::Ptr makeDefault(rw::common::Ptr<QConstraint> constraint,
			rw::common::Ptr< const rw::models::Device > device);


		/**
		 * @brief Makes an edge constraint by combining multiple edge constraints
		 *
		 * The constraints provided are called one by one in the order provided.
		 * It is assumed that all constraints matches the same device.
		 *
		 * @param constraints [in] List of constraints to check
		 * @return Pointer to the resulting QEdgeConstraint. Pointer has ownership.
		 **/
		static QEdgeConstraint::Ptr makeMerged(const std::vector<QEdgeConstraint::Ptr>& constraints);

		/**
		 * @brief Makes an edge constraint by combining two edge constraints
		 *
		 * The constraints provided are called one by one in the order provided.
		 * It is assumed that all constraints matches the same device.
		 *
		 * @param constraint1 [in] First constraint to check
		 * @param constraint2 [in] Second constraint to check
		 * @return Pointer to the resulting QEdgeConstraint. Pointer has ownership.
		 **/
		static QEdgeConstraint::Ptr makeMerged(QEdgeConstraint::Ptr constraint1, QEdgeConstraint::Ptr constraint2);


    protected:

        /**
           @brief Subclass implementation of the inCollision() method.

           By default the method is implemented in terms of instance() and
           inCollision().
        */
        virtual bool doInCollision(const rw::math::Q& start,
								   const rw::math::Q& end) const = 0;



    };

    /*@}*/
}} // end namespaces

#endif // end include guard
