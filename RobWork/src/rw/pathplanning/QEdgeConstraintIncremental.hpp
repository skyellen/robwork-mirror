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


#ifndef RW_PATHPLANNING_QEDGECONSTRAINTINCREMENTAL_HPP
#define RW_PATHPLANNING_QEDGECONSTRAINTINCREMENTAL_HPP

/**
   @file QEdgeConstraintIncremental.hpp
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
       @brief Edge constraint interface for incremental testing of an edge

       An edge constraint represents a path that connects a pair of
       configurations and checks if this path can be traversed.

       The edge constraint may assume that the start and end configurations are
       valid (e.g. not colliding).

       Each edge has a non-negative cost measuring the degree to which the path
       connecting the configurations has been verified. You can use the cost
       measure to for example always verify the edge for which the most of the
       path still remains to be verified. The exact meaning of the cost is
       defined by the specific subclass.

       Given an edge planner you can construct a new edge planner of the same
       type, but for a new pair of configurations, with
       QEdgeConstraint::instance().
    */
    class QEdgeConstraintIncremental
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QEdgeConstraintIncremental> Ptr;

        /**
           @brief Destructor
        */
        virtual ~QEdgeConstraintIncremental();

        /**
           @brief True if the path from \b start to \b end can't be traversed.

           @param start [in] Start configuration.
           @param end [in] End configuration.
        */
        bool inCollision(
            const rw::math::Q& start,
            const rw::math::Q& end) const;

        /**
           @brief True if the path connecting the start and end configuration
           can't be traversed.
        */
        bool inCollision();

        /**
           @brief Non-negative measure of the amount of the path that still
           remains to be verified.

           The exact definition of the cost is decided by the subclass.

           The cost of an edge should strictly decrease for every call of
           verifyIncrement().

           The cost of a fully verified edge can be 0, but does not have to be.
        */
        double inCollisionCost() const;

        /**
           @brief Perform a partial check of the path and return true if a
           collision was found.

           Full check of the path can be implemented in terms of a sequence of
           partial checks. The isFullyChecked() method returns true when there
           are no more partial checks to be done.
        */
        bool inCollisionPartialCheck();

        /**
           @brief True if the path has been fully checked.

           To check a path, either call inCollision() or repeatedly call
           inCollisionPartialCheck() until inCollisionPartialCheck() returns
           false or isFullyChecked() returns true.
        */
        bool isFullyChecked() const;

        /**
           @brief An edge constraint for a pair of configurations.

           @param start [in] Start configuration of path
           @param end [in] End configuration of path
        */
		QEdgeConstraintIncremental::Ptr instance(
            const rw::math::Q& start,
            const rw::math::Q& end) const;

        /**
           @brief The start configuration of the path.
        */
        const rw::math::Q& getStart() const { return _start; }

        /**
           @brief The end configuration of the path.
        */
        const rw::math::Q& getEnd() const { return _end; }

        /**
           @brief Reset the object to use a different pair of start and end
           configurations.
        */
        void reset(const rw::math::Q& start, const rw::math::Q& end);

        // Here we have the factory methods.

        /**
           @brief Discrete path verification for a linearly interpolated path.

           Linearly interpolate from \b start to \b end configuration until the
           distance between pairs of configurations is \b resolution when
           measured by \b metric. Verify each configuration by \b constraint.

           The cost is defined as the distance (measured by \b metric) between
           pairs of configurations currently verified by \b constraint.

           The metric must be well-behaved, i.e. linear.

           You can pass empty configurations as \b start and \b end to construct
           an initial edge planner that you can instance() with better
           configurations later.

           Start and end configurations for this initial planner are set to the
           empty configuration.
        */
		static QEdgeConstraintIncremental::Ptr make(
			rw::common::Ptr<QConstraint> constraint,
			rw::math::QMetric::Ptr metric,
            double resolution = 1);

        /**
           @brief Default edge constraint for a configuration constraint and a
           device.

           Start and end configurations are connected by a straight line in the
           configuration space and are checked by a default collision checking
           resolution.
        */
		static QEdgeConstraintIncremental::Ptr makeDefault(
			rw::common::Ptr<QConstraint> constraint,
			rw::common::Ptr<rw::models::Device> device);

        /**
           @brief A fixed edge constraint.

           The fixed edge constraint always returns \b value from inCollision().
        */
        static
			QEdgeConstraintIncremental::Ptr makeFixed(bool value);

        // We can implement a bunch of other instances, for example an instance
        // parameterized by an interpolator.

    protected:
        /**
           @brief Constructor provided for subclasses.

           @param start [in] Start configuration of path
           @param end [in] End configuration of path
        */
        QEdgeConstraintIncremental(
            const rw::math::Q& start,
            const rw::math::Q& end);

        /**
           @brief Subclass implementation of the inCollision() method.

           By default the method is implemented in terms of instance() and
           inCollision().
        */
        virtual bool doInCollision(
            const rw::math::Q& start,
            const rw::math::Q& end) const;

        /**
           @brief Subclass implementation of the inCollision() method.

           By default this method is implemented in terms of
           inCollisionPartialCheck() and isFullyChecked().
        */
        virtual bool doInCollision();

        /**
           @brief Subclass implementation of the inCollisionCost() method.
        */
        virtual double doInCollisionCost() const = 0;

        /**
           @brief Subclass implementation of the inCollisionPartialCheck() method.

           By default this method is implemented in terms of inCollision().
        */
        virtual bool doInCollisionPartialCheck();

        /**
           @brief Subclass implementation of the isFullyChecked() method.
        */
        virtual bool doIsFullyChecked() const = 0;

        /**
           @brief Subclass implementation of the instance() method.
        */
		virtual QEdgeConstraintIncremental::Ptr doClone(
            const rw::math::Q& start,
            const rw::math::Q& end) const = 0;

        /**
           @brief Subclass implementation of the reset() method.

           The start and end configurations will be reset before doReset() is
           called, and therefore the start and end configurations are not passed
           to doReset().
        */
        virtual void doReset() = 0;

    private:
        QEdgeConstraintIncremental(const QEdgeConstraintIncremental&);
        QEdgeConstraintIncremental& operator=(const QEdgeConstraintIncremental&);

    private:
        rw::math::Q _start;
        rw::math::Q _end;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
