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


#ifndef RW_PATHPLANNING_QIKSAMPLER_HPP
#define RW_PATHPLANNING_QIKSAMPLER_HPP

/**
   @file QIKSampler.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace invkin { class IterativeIK; } }

namespace rw { namespace pathplanning {
	class QConstraint;
	class QSampler;

    /** @addtogroup pathplanning */
    /** @{*/

    /**
       @brief Interface for the sampling a configuration that solves an IK
       problem.
    */
    class QIKSampler
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QIKSampler> Ptr;

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

           All solutions returned are checked to be within the bounds of the device.

           @param device [in] The device for which seeds are sampled.

           @param state [in] Fixed state with respect to which IK is solved.

           @param solver [in] Optional IK solver for \b device and \b state.

           @param seed [in] Optional sampler of seeds to feed the IK solver.

           @param maxAttempts [in] Optional number of seeds to feed the IK
           solver. If \b maxAttempts is negative, a default value for \b
           maxAttempts is chosen.
        */
		static QIKSampler::Ptr make(
			rw::common::Ptr<rw::models::Device> device,
            const rw::kinematics::State& state,
			rw::common::Ptr<rw::invkin::IterativeIK> solver = NULL,
			rw::common::Ptr<QSampler> seed = NULL,
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
        static QIKSampler::Ptr makeConstrained(
		    QIKSampler::Ptr sampler,
			rw::common::Ptr<QConstraint> constraint,
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
