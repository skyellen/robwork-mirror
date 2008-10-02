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

#ifndef rw_pathplanning_QIKSampler_HPP
#define rw_pathplanning_QIKSampler_HPP

/**
   @file QIKSampler.hpp
*/

#include "QSampler.hpp"
#include "QConstraint.hpp"
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/invkin/IterativeIK.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class QIKSampler;

    //! A pointer to a QIKSampler.
    typedef rw::common::Ptr<QIKSampler> QIKSamplerPtr;

    /**
       @brief Interface for the sampling a configuration that solves an IK
       problem.
    */
    class QIKSampler
    {
    public:
        /**
           @brief Sample a configuration that solves an IK problem for \b
           target.

           If sampling fails, the sampler may return the empty configuration. If
           empty() is true then the sampler has no more configurations.
           Otherwise sample() may (or may not) succeed if called a second time.
        */
        rw::math::Q sample(const rw::math::Transform3D<>& target)
        { return doSample(target); }

        /**
           @brief True if the sampler is known to contain no more
           configurations.
        */
        bool empty() const;

        /**
           @brief Destructor
        */
        virtual ~QIKSampler() {}

        /**
           @brief An IK sampler based on an iterative IK solver.

           @param device [in] The device for which seeds are sampled.

           @param state [in] Fixed state with respect to which IK is solved.

           @param solver [in] Optional IK solver for \b device and \b state.

           @param seed [in] Optional sampler of seeds to feed the IK solver.

           @param maxAttempts [in] Optional number of seeds to feed the IK
           solver. If \b maxAttempts is negative, a default value for \b
           maxAttempts is chosen.
        */
        static QIKSamplerPtr make(
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            rw::invkin::IterativeIKPtr solver = NULL,
            QSamplerPtr seed = NULL,
            int maxAttempts = -1);

        /**
           @brief An IK sampler filtered by a constraint.

           For each call of sample() up to \b maxAttempts configurations are
           extracted from \b sampler and checked by \b constraint. The first
           sample that satisfies the constraint is returned; if no such were
           found the empty configuration is returned.

           If \b maxAttempts is negative, then \b sampler is sampled forever
           until either the \b sampler is empty or a configuration satisfying \b
           constraint is found.
        */
        static
        QIKSamplerPtr makeConstrained(
            QIKSamplerPtr sampler,
            QConstraintPtr constraint,
            int maxAttempts = -1);

    protected:
        /**
           @brief Constructor
        */
        QIKSampler() {}

        /**
           @brief Subclass implementation of the sample() method.
        */
        virtual rw::math::Q doSample(
            const rw::math::Transform3D<>& target) = 0;

        /**
           @brief Subclass implementation of the empty() method.

           By default the sampler is assumed to be sampling an infinite set of
           configurations. IOW. the function returns false by default.
        */
        virtual bool doEmpty() const;

    private:
        QIKSampler(const QIKSampler&);
        QIKSampler& operator=(const QIKSampler&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
