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

#ifndef rw_pathplanning_QSampler_HPP
#define rw_pathplanning_QSampler_HPP

/**
   @file QSampler.hpp
*/

#include "QConstraint.hpp"
#include "QNormalizer.hpp"
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/invkin/IterativeIK.hpp>
#include <memory>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class QSampler;

    //! A pointer to a QSampler.
    typedef rw::common::Ptr<QSampler> QSamplerPtr;

    /**
       @brief Interface for the sampling a configuration.
    */
    class QSampler
    {
    public:
        /**
           @brief Sample a configuration.

           If sampling fails, the sampler may return the empty configuration. If
           empty() is true then the sampler has no more configurations.
           Otherwise sample() may (or may not) succeed if called a second time.
        */
        rw::math::Q sample() { return doSample(); }

        /**
           @brief True if the sampler is known to contain no more
           configurations.
        */
        bool empty() const;

        /**
           @brief Destructor
        */
        virtual ~QSampler() {}

        /**
           @brief Empty sampler.
        */
        static std::auto_ptr<QSampler> makeEmpty();

        /**
           @brief Sampler that always returns the same configuration.

           The sampler is considered never empty (empty() always returns false).
        */
        static std::auto_ptr<QSampler> makeFixed(const rw::math::Q& q);

        /**
           @brief Sampler that always returns a single configuration.

           The sample() returns \b q the first time the method is called and the
           empty configuration otherwise. empty() returns true after the first
           call of sample().
        */
        static std::auto_ptr<QSampler> makeSingle(const rw::math::Q& q);

        /**
           @brief Sampler for the values of a finite sequence.

           sample() returns each of the values of \b qs in order. When all of
           these samples have been returned, empty() returns true and sample()
           returns the empty configuration.
        */
        static std::auto_ptr<QSampler> makeFinite(const std::vector<rw::math::Q>& qs);

        /**
           @brief Uniform random sampling for a box of the configuration space.
        */
        static std::auto_ptr<QSampler> makeUniform(
            const std::pair<rw::math::Q, rw::math::Q>& bounds);

        /**
           @brief Uniform random sampling for a device.
        */
        static std::auto_ptr<QSampler> makeUniform(
            const rw::models::Device& device);

        /**
           @brief Uniform random sampling for a device.
        */
        static std::auto_ptr<QSampler> makeUniform(
            rw::models::DevicePtr device);

        /**
           @brief Map a sampler of standard configurations into a sampler of
           normalized configurations.
        */
        static std::auto_ptr<QSampler> makeNormalized(
            QSamplerPtr sampler,
            const QNormalizer& normalizer);

        /**
           @brief Sampling of IK solutions.

           The iterative IK solver is fed start configurations sampled uniformly
           at random for \b device.

           @param solver [in] IK solver.

           @param device [in] The device for which IK is solved.

           @param state [in] State of the rest of the workcell.

           @param baseTend [in] base to end transform for the IK solver to
           reach.

           @param maxAttempts [in] Maximum number of start configuration to use
           per call.
        */
        static std::auto_ptr<QSampler> makeIterativeIK(
            rw::invkin::IterativeIKPtr solver,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            const rw::math::Transform3D<>& baseTend,
            int maxAttempts);

        /**
           @brief Sampling of IK solutions.

           The iterative IK solver is fed start configurations sampled uniformly
           at random for \b device.

           The sampler assumes that \b state initially contains a valid
           solution.

           @param solver [in] IK solver.

           @param device [in] The device for which IK is solved.

           @param state [in] State of the rest of the workcell.

           @param maxAttempts [in] Maximum number of start configuration to use
           per call.
        */
        static std::auto_ptr<QSampler> makeIterativeIK(
            rw::invkin::IterativeIKPtr solver,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            int maxAttempts);

        /**
           @brief Sampling of IK solutions.

           The iterative IK solver is fed start configurations sampled uniformly
           at random for \b device.

           The sampler assumes that \b state initially contains a valid
           IK solution for \b device.

           A default iterative IK solver is chosen for \b device.

           @param device [in] The device for which IK is solved.

           @param state [in] State of the rest of the workcell.

           @param maxAttempts [in] Maximum number of start configuration to use
           per call.
        */
        static std::auto_ptr<QSampler> makeIterativeIK(
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            int maxAttempts);

        /**
           @brief A sampler filtered by a constraint.

           For each call of sample() up to maxAttempts configurations are
           extracted from \b sampler and checked by \b constraint. The first
           sample that satisfies the constraint is returned; if no such were
           found the empty configuration is returned.
        */
        static std::auto_ptr<QSampler> makeConstrained(
            QSamplerPtr sampler,
            QConstraintPtr constraint,
            int maxAttempts);

    protected:
        /**
           @brief Constructor
        */
        QSampler() {}

        /**
           @brief Subclass implementation of the sample() method.
        */
        virtual rw::math::Q doSample() = 0;

        /**
           @brief Subclass implementation of the empty() method.

           By default the sampler is assumed to be sampling an infinite set of
           configurations. IOW. the function returns false by default.
        */
        virtual bool doEmpty() const;

    private:
        QSampler(const QSampler&);
        QSampler& operator=(const QSampler&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
