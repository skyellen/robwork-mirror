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

#ifndef rw_pathplanning_QConstraint_hpp
#define rw_pathplanning_QConstraint_hpp

/**
   @file QConstraint.hpp
*/

#include "StateConstraint.hpp"
#include "QNormalizer.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class QConstraint;

    //! A pointer to a QConstraint.
    typedef rw::common::Ptr<QConstraint> QConstraintPtr;

    /**
       @brief Interface for the checking for collisions for work cell states.
    */
    class QConstraint
    {
    public:
        /**
           @brief True if the work cell is considered to be in collision for the
           device configuration \b q.
         */
        bool inCollision(const rw::math::Q& q) const;

        /**
           @brief Destructor
        */
        virtual ~QConstraint() {}

        /**
           @brief A fixed constraint.

           The fixed constraint always returns \b value from inCollision().
        */
        static std::auto_ptr<QConstraint> makeFixed(bool value);

        /**
           @brief Map a state constraint to a configuration constraint.
        */
        static std::auto_ptr<QConstraint> make(
            StateConstraintPtr detector,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

        /**
           @brief Map a collision detector to a configuration constraint.
        */
        static std::auto_ptr<QConstraint> make(
            rw::proximity::CollisionDetectorPtr detector,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

        /**
           @brief Combine a set of configuration constraints into a single
           configuration constraint.
         */
        static std::auto_ptr<QConstraint> makeMerged(
            const std::vector<QConstraintPtr>& constraints);

        /**
           @brief Combine a pair of configuration constraints into a single
           configuration constraint.
         */
        static std::auto_ptr<QConstraint> makeMerged(
            const QConstraintPtr& ca,
            const QConstraintPtr& cb);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from the range [0, 1] into the
           corresponding position in the box \b bounds.
        */
        static std::auto_ptr<QConstraint> makeNormalized(
            const QConstraintPtr& constraint,
            const std::pair<rw::math::Q, rw::math::Q>& bounds);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from the range [0, 1] into the
           corresponding position in the configuration space of \b device.
        */
        static std::auto_ptr<QConstraint> makeNormalized(
            const QConstraintPtr& constraint,
            const rw::models::Device& device);

        /**
           @brief Map a configuration constraint for standard configurations
           into a configuration constraint for normalized configurations.

           Configuration values are mapped from normalized configurations into
           standard configurations using \b normalizer.
        */
        static std::auto_ptr<QConstraint> makeNormalized(
            const QConstraintPtr& constraint,
            const QNormalizer& normalizer);

    protected:
        /**
           @brief Subclass implementation of the inCollision() method.
        */
        virtual bool doInCollision(const rw::math::Q& q) const = 0;

        /**
           Constructor
        */
        QConstraint() {}

    private:
        QConstraint(const QConstraint&);
        QConstraint& operator=(const QConstraint&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
