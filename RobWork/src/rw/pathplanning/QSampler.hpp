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


#ifndef RW_PATHPLANNING_QSAMPLER_HPP
#define RW_PATHPLANNING_QSAMPLER_HPP

/**
   @file QSampler.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace pathplanning {
	class QConstraint;
	class QNormalizer;

    /** @addtogroup pathplanning */
    /** @{*/

    // Forward declaration.
    class QIKSampler;

    /**
       @brief Interface for the sampling a configuration.
    */
    class QSampler
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QSampler> Ptr;
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr< const QSampler > CPtr;

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
		static QSampler::Ptr makeEmpty();

        /**
           @brief Sampler that always returns the same configuration.

           The sampler is considered never empty (empty() always returns false).
        */
		static QSampler::Ptr makeFixed(const rw::math::Q& q);

        /**
           @brief Sampler that always returns a single configuration.

           The sample() returns \b q the first time the method is called and the
           empty configuration otherwise. empty() returns true after the first
           call of sample().
        */
		static QSampler::Ptr makeSingle(const rw::math::Q& q);

        /**
           @brief Sampler for the values of a finite sequence.

           sample() returns each of the values of \b qs in order. When all of
           these samples have been returned, empty() returns true and sample()
           returns the empty configuration.
        */
		static QSampler::Ptr makeFinite(const std::vector<rw::math::Q>& qs);

        /**
           @brief A sampler to that returns only the first \b cnt samples from
           another sampler.

           The sampler is considered empty as soon as \b sampler is empty or the
           sampler has been called \b cnt times or more.
        */
		static QSampler::Ptr makeFinite(QSampler::Ptr sampler, int cnt);

        /**
           @brief Uniform random sampling for a box of the configuration space.
        */
		static QSampler::Ptr makeUniform(
            const rw::models::Device::QBox& bounds);

        /**
           @brief Uniform random sampling for a device.
        */
		static QSampler::Ptr makeUniform(
            const rw::models::Device& device);

        /**
           @brief Uniform random sampling for a device.
        */
		static QSampler::Ptr makeUniform(rw::models::Device::Ptr device);

        /**
           @brief Map a sampler of standard configurations into a sampler of
           normalized configurations.
        */
		static QSampler::Ptr makeNormalized(QSampler::Ptr sampler,
            const QNormalizer& normalizer);

        /**
           @brief A sampler of IK solutions for a specific target.

           @param sampler [in] Sampler of IK solutions for \b target.
           @param target [in] Target for IK solver.
        */
		static QSampler::Ptr make(rw::common::Ptr<QIKSampler> sampler,
            const rw::math::Transform3D<>& target);

        /**
           @brief A sampler filtered by a constraint.

           For each call of sample() up to \b maxAttempts configurations are
           extracted from \b sampler and checked by \b constraint. The first
           sample that satisfies the constraint is returned; if no such were
           found the empty configuration is returned.

           If \b maxAttempts is negative (this is the default), then \b sampler
           is sampled forever until either the \b sampler is empty or a
           configuration satisfying \b constraint is found.
        */
		static QSampler::Ptr makeConstrained(
			QSampler::Ptr sampler,
			rw::common::Ptr<QConstraint> constraint,
            int maxAttempts = -1);

        /**
           @brief Sampler of direction vectors for a box shaped configuration
           space.

           Each random direction vector is found by sampling a configuration
           uniformly at random from \b bounds, and returning the vector from the
           center of the box to the configuration. The returned direction vector
           can therefore be of length zero.
        */
        static
			QSampler::Ptr makeBoxDirectionSampler(
            const rw::models::Device::QBox& bounds);

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
