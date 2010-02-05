/*
 * StateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef STATESAMPLER_HPP_
#define STATESAMPLER_HPP_

#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>

/**
 * @brief interface for generating states.
 */
class StateSampler {
    public:
        /**
           @brief Sample a state.

           If sampling fails, the sampler may return the empty configuration. If
           empty() is true then the sampler has no more configurations.
           Otherwise sample() may (or may not) succeed if called a second time.
        */
        virtual bool sample(rw::kinematics::State& state) = 0;

        /**
           @brief True if the sampler is known to contain no more
           configurations.
        */
        virtual bool empty() const = 0;
};

typedef rw::common::Ptr<StateSampler> StateSamplerPtr;

#endif /* STATESAMPLER_HPP_ */
