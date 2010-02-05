/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef COLLISIONFREESAMPLER_HPP_
#define COLLISIONFREESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/proximity/CollisionDetector.hpp>

/**
 * @brief samples another state sampler until it returns a collision free
 * state.
 */
class CollisionFreeSampler: public StateSampler
{
public:

    /**
     * @brief
     * @param sampler [in]
     * @param detector [in]
     * @param n [in]
     * @return
     */
    CollisionFreeSampler(StateSamplerPtr sampler, rw::proximity::CollisionDetectorPtr detector, int n=-1);

    virtual ~CollisionFreeSampler();


    bool sample(rw::kinematics::State& state);

    bool empty() const{ return false; };

private:
    StateSamplerPtr _sampler;
    rw::proximity::CollisionDetectorPtr _detector;
    int _n;

};

#endif /* FINITESTATESAMPLER_HPP_ */
