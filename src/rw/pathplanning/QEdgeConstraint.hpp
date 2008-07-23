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

#ifndef rw_pathplanning_QEdgeConstraint_HPP
#define rw_pathplanning_QEdgeConstraint_HPP

/**
   @file QEdgeConstraint.hpp
*/

#include "Path.hpp"
#include "QConstraint.hpp"
#include "StopCriteria.hpp"
#include <rw/common/PropertyMap.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Metric.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    class QEdgeConstraint;

    //! A pointer to a QEdgeConstraint.
    typedef rw::common::Ptr<QEdgeConstraint> QEdgeConstraintPtr;

    /**
       @brief Edge planner interface.

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
    class QEdgeConstraint
    {
    public:
        /**
           @brief Destructor
        */
        virtual ~QEdgeConstraint();

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
        std::auto_ptr<QEdgeConstraint> instance(
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
        static std::auto_ptr<QEdgeConstraint> make(
            QConstraintPtr constraint,
            rw::math::MetricPtr metric,
            double resolution = 1);

        /**
           @brief Default edge constraint for a configuration constraint and a
           device.

           Start and end configurations are connected by a straight line in the
           configuration space and are checked by a default collision checking
           resolution.
        */
        static std::auto_ptr<QEdgeConstraint> makeDefault(
            QConstraintPtr constraint,
            rw::models::DevicePtr device);

        // We can implement a bunch of other instances, for example an instance
        // parameterized by an interpolator.

    protected:
        /**
           @brief Constructor provided for subclasses.

           @param start [in] Start configuration of path
           @param end [in] End configuration of path
        */
        QEdgeConstraint(
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
        virtual std::auto_ptr<QEdgeConstraint> doClone(
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
        QEdgeConstraint(const QEdgeConstraint&);
        QEdgeConstraint& operator=(const QEdgeConstraint&);

    private:
        rw::math::Q _start;
        rw::math::Q _end;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
